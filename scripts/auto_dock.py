#!/usr/bin/env python

# Author: Jack Kilian
# Description: This script auto_docks the openrover basic platform if it is in a 3m circle in front of the dock.

import numpy as np
import os
import time
import math

import rospy
import actionlib
from std_msgs.msg import Float32, String, Bool, Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import *

import rr_auto_dock.msg

from wibotic_msg.srv import ReadParameter

WIBOTICS_ANTENNA_DETECTED = 1409286812
MANAGER_PERIOD = 0.1
START_DELAY = 2.0  # is this necessary?
MIN_TURN_PERIOD = 0.18
MAX_RUN_TIMEOUT = 240  # in seconds
ARUCO_WAIT_TIMEOUT = 2  # in seconds
Z_TRANS_OFFSET = 0  # 0.5
DOCKING_STATE_LIST = {
    "undocked",
    "searching",
    "centering",
    "approach",
    "final_approach",
    "waiting_for_in_range",
    "docking_failed",
    "docked",
    "undocking",
}
ACTION_STATE_LIST = {
    "turning",
    "count_aruco_callbacks",
    "jogging",
    "stopping",
    "waiting",
}
UNDOCKING_STATE_LIST = {"reversing", "turning"}


class ArucoDockingManager(object):
    def __init__(self):

        # get rosparam constants
        self.cmd_vel_angular_rate = rospy.get_param(
            "~cmd_vel_angular_rate"
        )  # rad/s, negative is clockwise
        self.cmd_vel_linear_rate = rospy.get_param("~cmd_vel_linear_rate")  # m/s
        self.turn_radians = rospy.get_param("~turn_radians")
        self.approach_angle = rospy.get_param("~approach_angle")  # max approach angle
        self.approach_radius = rospy.get_param(
            "~approach_radius"
        )  # radius that uses max corrective angle
        self.aruco_callback_counter_max = rospy.get_param("~aruco_callback_counter_max")
        self.max_centering_count = rospy.get_param("~max_centering_count")
        self.jog_distance = rospy.get_param("~jog_distance")
        self.final_approach_distance = rospy.get_param("~final_approach_distance")
        self.undock_distance = rospy.get_param("~undock_distance")
        self.in_range_wait_time = rospy.get_param("~in_range_wait_time")
        # travel distance is achieved by sending vel commands for [distance/linear_rate] seconds
        # this is a correction based on testing; would need to be tuned for any given scenario
        self.open_loop_correction = rospy.get_param("~open_loop_correction")

        # initialize variables
        self.aruco_callback_counter = 0
        self.centering_counter = 0
        self.cmd_vel_angular = 0
        self.cmd_vel_linear = 0
        self.cmd_vel_msg = TwistStamped()
        self.is_in_view = False  # aruco marker detected
        self.is_in_antenna_range = False  # charging station within coil range
        self.aruco_last_time = rospy.Time()
        self.last_dock_aruco_tf = Transform()
        self.dock_aruco_tf = Transform()
        self.action_state = ""
        self.action_state_data = ""
        self.action_state_msg = String()
        self.undocking_state = ""
        self.docking_state = "undocked"  # initial state
        self.docking_state_msg = String()
        self.docking_state_msg.data = self.docking_state
        self.last_docking_state = ""
        self.last_action_state = ""

        rospy.loginfo("autodock node running")

        # Publishers
        self.pub_aruco_detections_enable = rospy.Publisher(
            "/aruco_detect/enable", Bool, queue_size=1, latch=True
        )
        self.pub_cmd_vel = rospy.Publisher(
            "/cmd_vel/auto_dock", TwistStamped, queue_size=1, latch=True
        )
        self.pub_docking_state = rospy.Publisher(
            "/auto_dock/state", String, queue_size=1, latch=True
        )
        self.pub_action_state = rospy.Publisher(
            "/auto_dock/action_state", String, queue_size=1, latch=True
        )
        self.pub_docking_state.publish(self.docking_state_msg)

        # Subscribers
        self.sub_aruco_detect = rospy.Subscriber(
            "/fiducial_transforms",
            FiducialTransformArray,
            self.aruco_detect_cb,
            queue_size=1,
        )

        # service for determining whether we're in range of the charging station
        rospy.loginfo("Waiting for service wibotic_connector_can/read_parameter...")
        try:
            rospy.wait_for_service("wibotic_connector_can/read_parameter", timeout=10)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn(
                "%s: Autodocking node timed out waiting for wibotic service", e
            )
        self.in_antenna_range_service = rospy.ServiceProxy(
            "wibotic_connector_can/read_parameter", ReadParameter
        )

        self.action_server_rate = 1  # Hz
        # setup dock action server
        self._dock_feedback = rr_auto_dock.msg.AutoDockFeedback()
        self._dock_result = rr_auto_dock.msg.AutoDockResult()
        self._dock_action_server = actionlib.SimpleActionServer(
            "/auto_dock/dock",
            rr_auto_dock.msg.AutoDockAction,
            execute_cb=self.execute_dock_cb,
            auto_start=False,
        )
        self._dock_action_server.start()
        # setup undock action server
        self._undock_feedback = rr_auto_dock.msg.AutoDockFeedback()
        self._undock_result = rr_auto_dock.msg.AutoDockResult()
        self._undock_action_server = actionlib.SimpleActionServer(
            "/auto_dock/undock",
            rr_auto_dock.msg.AutoDockAction,
            execute_cb=self.execute_undock_cb,
            auto_start=False,
        )
        self._undock_action_server.start()

        # Setup main timer/frequency for state machine
        self.state_manager_timer = rospy.Timer(
            rospy.Duration(MANAGER_PERIOD), self.state_manage_cb, oneshot=False
        )

    def state_manage_cb(self, event):
        # rospy.loginfo("%s | %s", self.docking_state, self.last_docking_state)
        if self.docking_state == "undocking":
            self.disable_aruco_detections()
            self.undock_state_fun()

        if self.docking_state == "undocked":
            self.disable_aruco_detections()
            self.undocked_state_fun()
            # self.in_antenna_range_service_call() #TODO determine whether this should be handled by task broker

        if self.docking_state == "searching":
            self.enable_aruco_detections()
            self.searching_state_fun()

        if self.docking_state == "centering":
            self.enable_aruco_detections()
            self.centering_state_fun()

        if self.docking_state == "approach":
            self.enable_aruco_detections()
            self.approach_state_fun()
            self.in_antenna_range_service_call()

        if self.docking_state == "final_approach":
            self.enable_aruco_detections()
            self.final_approach_state_fun()
            self.in_antenna_range_service_call()

        if self.docking_state == "waiting_for_in_range":
            self.disable_aruco_detections()
            self.waiting_for_in_range_state_fun()
            self.in_antenna_range_service_call()

        if self.docking_state == "docked":
            self.disable_aruco_detections()
            # self.in_antenna_range_service_call()  #TODO determine whether this should be handled by task broker

        if self.docking_state == "docking_failed":
            self.disable_aruco_detections()

        if self.docking_state == "cancelled":
            self.disable_aruco_detections()
            self.cancelled_state_fun()

        action_state_data = "%s | %s" % (self.docking_state, self.action_state)
        self.publish_action_state(action_state_data)
        self.publish_docking_state()
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

    def set_docking_state(self, new_docking_state):
        if not self.docking_state == new_docking_state:
            self.last_docking_state = self.docking_state
            self.docking_state = new_docking_state
            # rospy.loginfo(
            #     "new state: %s, last state: %s",
            #     self.docking_state,
            #     self.last_docking_state,
            # )

    def set_action_state(self, new_action_state):
        if not self.action_state == new_action_state:
            self.last_action_state = self.action_state
            self.action_state = new_action_state
            # rospy.loginfo("new astate: %s, last astate: %s", self.action_state, self.last_action_state)

    def undocked_state_fun(self):
        self.set_action_state("")

    # TODO - this doesn't seem to have an exit path if a marker isn't found
    # need to implement a timeout or something
    def searching_state_fun(self):
        # rospy.loginfo("searching aruco count: %i", self.aruco_callback_counter)
        self.centering_counter = 0
        if self.action_state == "turning":
            return
        if self.aruco_callback_counter < self.aruco_callback_counter_max:
            self.set_action_state("count_aruco_callbacks")
        else:
            self.aruco_callback_counter = 0
            self.set_action_state("")
            self.openrover_turn(-self.turn_radians)

    def centering_state_fun(self):
        # wait for another detection then center
        if self.action_state == "turning":
            return
        if self.aruco_callback_counter < 1:
            self.centering_counter = self.centering_counter + 1
            self.set_action_state("count_aruco_callbacks")
            return
        self.aruco_callback_counter = 0
        self.set_action_state("")
        if self.centering_counter >= self.max_centering_count:
            rospy.logwarn(
                "centering failed. reverting to last state: %s", self.last_docking_state
            )
            self.aruco_callback_counter = 0
            self.set_action_state("")
            self.set_docking_state(self.last_docking_state)
            return
        if self.is_in_view:
            [theta, distance, theta_bounds] = self.fid2pos(self.dock_aruco_tf)
            if abs(theta) > theta_bounds:
                self.openrover_turn(theta)
            else:
                rospy.loginfo("centered switching to approach state")
                self.set_docking_state("approach")
                self.openrover_stop()

    def approach_state_fun(self):
        self.centering_counter = 0
        if self.is_in_view:
            [theta, distance, theta_bounds] = self.fid2pos(self.dock_aruco_tf)
            if abs(theta) > theta_bounds:
                rospy.loginfo("approach angle exceeded: %f", abs(theta))
                self.openrover_stop()
                self.set_docking_state("centering")
            else:
                if self.action_state == "jogging":
                    # continuing moving forward in 'approach' state using 'jog distance'
                    # until we get within 'final approach' distance
                    return
                if abs(distance) < self.final_approach_distance:
                    # within final approach distance; move forward remaining distance
                    self.openrover_stop()
                    self.openrover_forward(abs(distance))
                    self.set_docking_state("final_approach")
                else:
                    # not yet within final approach distance
                    # creep forward some distance >= minimum jog distance
                    # depending on how far away you are
                    self.openrover_forward(
                        np.maximum(
                            (abs(distance) - self.final_approach_distance) / 2,
                            self.jog_distance,
                        )
                    )
        else:
            # too close to the marker to detect anymore; proceed with final approach
            self.openrover_stop()
            self.openrover_forward(self.final_approach_distance)
            self.set_docking_state("final_approach")

    def final_approach_state_fun(self):
        [theta, distance, theta_bounds] = self.fid2pos(self.dock_aruco_tf)
        if self.is_in_view and abs(distance) > self.final_approach_distance:
            self.openrover_stop()
            self.set_docking_state("approach")
            return
        if self.action_state == "jogging":
            return
        if self.action_state == "":
            # the linear timer for the forward command called in approach_state_fun
            # has timed out and the rover has stopped (presumably adjacent to charger)
            self.set_docking_state("waiting_for_in_range")

    def waiting_for_in_range_state_fun(self):
        # sometimes can take a couple seconds for the in_antenna_range
        # confirmation to be received
        if self.action_state == "":
            rospy.loginfo("Waiting for antenna detection")
            self.set_action_state("waiting")
            self.waiting_timer = rospy.Timer(
                rospy.Duration(self.in_range_wait_time),
                self.docking_failed_cb,
                oneshot=True,
            )

    def undock_state_fun(self):
        if self.action_state == "jogging":
            return
        if self.action_state == "turning":
            return
        if self.undocking_state == "":
            rospy.logwarn("Backup")
            self.openrover_forward(-self.undock_distance)
            self.undocking_state = "reversing"
            return
        if self.undocking_state == "reversing":
            rospy.logwarn("Undock turning")
            self.openrover_turn(3.0)
            self.undocking_state = "turning"
            return
        self.set_docking_state("undocked")

    def cancelled_state_fun(self):
        self.openrover_stop()
        self.full_reset()
        self.set_docking_state("undocked")

    def publish_docking_state(self):  # Publish docking state if it has changed
        if not (self.docking_state_msg.data == self.docking_state):
            self.docking_state_msg.data = self.docking_state
            self.pub_docking_state.publish(self.docking_state_msg)

    def publish_action_state(
        self, string_in
    ):  # Publish docking state if it has changed
        self.action_state_msg.data = string_in
        self.pub_action_state.publish(self.action_state_msg)

    def full_reset(self):  # reset all variables to startup conditions
        self.cmd_vel_angular = 0
        self.cmd_vel_linear = 0
        self.is_in_view = False
        self.is_in_antenna_range = False

        self.aruco_last_time = rospy.Time()
        self.set_action_state("")
        self.undocking_state = ""
        self.centering_counter = 0
        try:
            self.docking_timer.shutdown()
            rospy.loginfo("full_reset shutdown docking_timer: ")
        except:
            pass
        try:
            self.undocked_timer.shutdown()
            rospy.loginfo("full_reset shutdown undocked_timer: ")
        except:
            pass

    def openrover_forward(self, distance):
        if self.action_state == "":
            self.set_action_state("jogging")
            jog_period = (
                abs(distance) / self.cmd_vel_linear_rate * self.open_loop_correction
            )
            rospy.loginfo("jog_period: %f", jog_period)

            # this timer callback handles stopping at end of jog period
            # - calls openrover_stop, which sets vel cmds to 0 and action state to ""
            self.linear_timer = rospy.Timer(
                rospy.Duration(jog_period), self.openrover_linear_timer_cb, oneshot=True
            )

            if distance > 0:
                rospy.loginfo("Moving forward")
                self.cmd_vel_linear = self.cmd_vel_linear_rate
            else:
                rospy.loginfo("Moving Backward")
                # self.cmd_vel_linear = -self.cmd_vel_linear_rate * 1.5
                self.cmd_vel_linear = -self.cmd_vel_linear_rate
        self.cmd_vel_msg.twist.linear.x = self.cmd_vel_linear

    def fid2pos(self, fid_tf):
        q_now = [
            fid_tf.transform.rotation.x,
            fid_tf.transform.rotation.y,
            fid_tf.transform.rotation.z,
            fid_tf.transform.rotation.w,
        ]
        euler_angles = euler_from_quaternion(q_now)
        x_trans = fid_tf.transform.translation.x
        z_trans = fid_tf.transform.translation.z
        z_trans = z_trans - Z_TRANS_OFFSET
        theta = math.atan2(x_trans, z_trans)
        r = math.sqrt(x_trans ** 2 + z_trans ** 2)
        # approach angle is some maximum amount of turning to center
        if r > self.approach_radius:
            theta_bounds = self.approach_angle
        else:
            theta_bounds = r / self.approach_radius * self.approach_angle
        # rospy.loginfo(
        #     "Theta: %3.3f, r: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds
        # )
        return theta, r, theta_bounds

    def openrover_stop(self):
        self.set_action_state("")
        self.cmd_vel_msg.twist.linear.x = 0
        self.cmd_vel_msg.twist.angular.z = 0
        try:
            self.linear_timer.shutdown()
        except:
            pass
        try:
            self.turn_timer.shutdown()
        except:
            pass

    def openrover_turn(self, radians):
        if self.action_state == "":
            self.set_action_state("turning")
            turn_period = abs(radians / self.cmd_vel_angular_rate)
            if turn_period < MIN_TURN_PERIOD:
                turn_period = MIN_TURN_PERIOD

            # this timer callback handles stopping at end of turn period
            # - calls openrover_stop, which sets vel cmds to 0 and action state to ""
            self.turn_timer = rospy.Timer(
                rospy.Duration(turn_period), self.openrover_turn_timer_cb, oneshot=True
            )

            if radians > 0:
                rospy.loginfo("Turn right for %f", turn_period)
                self.cmd_vel_angular = -self.cmd_vel_angular_rate
            else:
                rospy.loginfo("Turn Left for %f", turn_period)
                self.cmd_vel_angular = self.cmd_vel_angular_rate
        rospy.loginfo("cmd_vel_angular = %s", self.cmd_vel_angular)
        self.cmd_vel_msg.twist.angular.z = self.cmd_vel_angular

    def disable_aruco_detections(self):
        disable_msg = Bool()
        disable_msg.data = False
        self.pub_aruco_detections_enable.publish(disable_msg)

    def enable_aruco_detections(self):
        enable_msg = Bool()
        enable_msg.data = True
        self.pub_aruco_detections_enable.publish(enable_msg)

    ##---Callbacks
    def execute_dock_cb(self, goal):
        """execute dock action"""
        success = True
        r = rospy.Rate(self.action_server_rate)

        rospy.loginfo("starting docking")
        self.openrover_stop()
        rospy.sleep(START_DELAY)  # TODO is this really necessary?
        if not self.docking_state == "docked":
            self.set_docking_state("searching")
            self.docking_timer = rospy.Timer(
                rospy.Duration(MAX_RUN_TIMEOUT), self.docking_failed_cb, oneshot=True,
            )

        while not self.docking_state == "docked" and not rospy.is_shutdown():
            self._dock_feedback.docking_state = self.docking_state
            self._dock_action_server.publish_feedback(self._dock_feedback)

            if self._dock_action_server.is_preempt_requested():
                rospy.loginfo("docking cancelled")
                self.set_docking_state("cancelled")

            if self.docking_state in ["docking_failed", "cancelled"]:
                success = False
                break
            r.sleep()

        self._dock_result.complete = success
        self._dock_action_server.set_succeeded(self._dock_result)

    def execute_undock_cb(self, goal):
        """execute undock action"""
        success = True
        r = rospy.Rate(self.action_server_rate)

        rospy.loginfo("starting undocking")
        if not self.docking_state == "cancelled":
            self.openrover_stop()
            self.full_reset()
            self.set_docking_state("undocking")
            self.set_action_state("")

        while not self.docking_state == "undocked" and not rospy.is_shutdown():
            self._undock_feedback.docking_state = self.docking_state
            self._undock_action_server.publish_feedback(self._undock_feedback)
            # TODO: currently there is no undocking failure path; maybe that's fine?
            r.sleep()

        self._undock_result.complete = success
        self._undock_action_server.set_succeeded(self._undock_result)

    def wait_now_cb(self, event):
        rospy.loginfo("wait_now_cb")
        if not (self.docking_state in ["docked", "waiting_for_in_range"]) and not (
            self.docking_state == "cancelled"
        ):
            self.set_docking_state("undocked")
            self.openrover_stop()
            self.docking_timer.shutdown()

    def aruco_detect_cb(self, fid_tf_array):
        if not (self.docking_state in ["docked", "waiting_for_in_range"]) and not (
            self.docking_state == "cancelled"
        ):

            # If no aruco cb's happen within ARUCO_WAIT_TIMEOUT seconds,
            # then assume the image pipe has been disconnected and go into undocked state
            try:
                self.undocked_timer.shutdown()
                self.undocked_timer = rospy.Timer(
                    rospy.Duration(ARUCO_WAIT_TIMEOUT), self.wait_now_cb, oneshot=True,
                )
            except:
                self.undocked_timer = rospy.Timer(
                    rospy.Duration(ARUCO_WAIT_TIMEOUT), self.wait_now_cb, oneshot=True,
                )

            if (
                self.action_state == "count_aruco_callbacks"
            ):  # pause while looking for a certain number of images
                self.aruco_callback_counter = self.aruco_callback_counter + 1
            else:
                self.aruco_callback_counter = 0
            try:
                # If there is no 0 index of transform, then aruco was not found
                fid_tf = fid_tf_array.transforms[0]
                self.last_dock_aruco_tf = self.dock_aruco_tf
                self.dock_aruco_tf = fid_tf
                self.is_in_view = True
                # rospy.loginfo("MARKER DETECTED!")
                [theta, r, theta_bounds] = self.fid2pos(
                    self.dock_aruco_tf
                )  # for debugging
                if self.docking_state == "searching":
                    self.openrover_stop()
                    self.aruco_callback_counter = 0
                    self.set_docking_state("centering")
            except:
                self.is_in_view = False

    def openrover_turn_timer_cb(self, event):
        rospy.loginfo("openrover_turn_timer_cb: Turning ended")
        self.openrover_stop()

    def openrover_linear_timer_cb(self, event):
        rospy.loginfo("openrover_linear_timer_cb: Stop moving forward")
        self.openrover_stop()

    def in_antenna_range_service_call(self):
        try:
            data = self.in_antenna_range_service(name="CCHK").value
            # rospy.loginfo("wibotics service call gave: %s", data)
            if data == WIBOTICS_ANTENNA_DETECTED:
                self.is_in_antenna_range = True
            self.docked_fun()
        except rospy.ServiceException as e:
            rospy.logwarn("error calling wibotic connector service: %s", e)

    def docked_fun(self):
        if self.docking_state == "undocking":
            return
        if self.is_in_antenna_range:
            if not self.docking_state == "docked":
                self.full_reset()
                self.openrover_stop()
                self.stop_waiting()
                self.set_docking_state("docked")
                rospy.loginfo("DOCKING SUCCESSFUL")
        else:
            if self.docking_state == "docked":
                self.set_docking_state("undocked")

    def stop_waiting(self):
        rospy.loginfo("shutting down wait-for-in-range timer")
        if self.action_state == "waiting":
            self.set_action_state("")
        try:
            self.waiting_timer.shutdown()
        except:
            pass

    def docking_failed_cb(self, event):
        rospy.loginfo("Docking failed cb")
        self.full_reset()
        self.openrover_stop()
        self.stop_waiting()
        self.set_docking_state("docking_failed")


def auto_dock_main():
    docking_manager = ArucoDockingManager()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()


if __name__ == "__main__":
    try:
        # Initialize docking node
        rospy.init_node("auto_dock", anonymous=True)
        auto_dock_main()
    except rospy.ROSInterruptException:
        pass
