#!/usr/bin/env python

# Author: Jack Kilian
# Description: This script auto_docks the openrover basic platform if it is in a 3m circle in front of the dock.

import numpy as np
import os
import time
import math

import rospy
from std_msgs.msg import Float32, String, Bool, Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray
from wibotic_msg.srv import ReadParameter
from rr_auto_dock.msg import dockAction, undockAction

# from tf.msgs import tf
from tf.transformations import *


class ArucoDockingManager(object):
    check_for_aruco = False
    aruco_callback_counter = 0
    centering_counter = 0
    cmd_vel_angular = 0
    cmd_vel_linear = 0
    cmd_vel_msg = TwistStamped()
    is_in_view = False  # aruco marker detected
    is_charging = False
    is_in_range = False  # charging station within coil range

    # don't think these booleans are actually used
    is_in_action = False
    is_final_jog = False
    is_turning = False
    is_looking = False
    is_jogging = False
    is_undocked = True
    is_undocking = False
    docking_failed = False

    aruco_last_time = rospy.Time()
    last_dock_aruco_tf = Transform()
    dock_aruco_tf = Transform()
    docking_state_list = {
        "undocked",
        "searching",
        "centering",
        "approach",
        "final_approach",
        "wait_for_charge",
        "docking_failed",
        "docked",
        "undock",
    }
    action_state_list = {
        "turning",
        "count_aruco_callbacks",
        "jogging",
        "stopping",
        "waiting",
    }
    action_state = ""
    action_state_data = ""
    action_state_msg = String()
    undocking_state_list = {"reversing", "turning"}
    undocking_state = ""
    docking_state = "undocked"  # initial state
    docking_state_msg = String()
    docking_state_msg.data = docking_state
    last_docking_state = ""
    last_action_state = ""

    def __init__(self):

        # rad/s, negative is clockwise
        self.CMD_VEL_ANGULAR_RATE = rospy.get_param("~cmd_vel_angular_rate")
        self.CMD_VEL_LINEAR_RATE = rospy.get_param("~cmd_vel_linear_rate")  # m/s
        self.TURN_RADIANS = rospy.get_param("~turn_radians")
        self.TURN_DURATION = abs(self.TURN_RADIANS / self.CMD_VEL_ANGULAR_RATE)
        self.APPROACH_ANGLE = rospy.get_param("~approach_angle")  # max approach angle
        self.APPROACH_RADIUS = rospy.get_param(
            "~approach_radius"
        )  # radius that uses max corrective angle
        self.ARUCO_CALLBACK_COUNTER_MAX = rospy.get_param("~aruco_callback_counter_max")
        self.MAX_CENTERING_COUNT = rospy.get_param("~max_centering_count")
        self.JOG_DISTANCE = rospy.get_param("~jog_distance")
        self.FINAL_APPROACH_DISTANCE = rospy.get_param("~final_approach_distance")
        self.UNDOCK_DISTANCE = rospy.get_param("~undock_distance")
        self.CHARGE_WAIT_TIME = rospy.get_param("~charge_wait_time")
        # travel distance is achieved by sending vel commands for [distance/linear_rate] seconds
        # this is a correction based on testing; would need to be tuned for any given scenario
        self.open_loop_correction = rospy.get_param("~open_loop_correction")

        self.MANAGER_PERIOD = 0.1
        self.START_DELAY = 2.0  # is this necessary?
        self.MIN_TURN_PERIOD = 0.18
        self.MAX_RUN_TIMEOUT = 240  # in seconds
        self.ARUCO_SLOW_WARN_TIMEOUT = rospy.Duration(1)  # in seconds
        self.ARUCO_WAIT_TIMEOUT = 2  # in seconds
        self.CANCELLED_TIMEOUT = 10  # in seconds
        self.Z_TRANS_OFFSET = 0  # 0.5

        rospy.loginfo("Starting automatic docking.")

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
        self.sub_openrover_charging = rospy.Subscriber(
            "rr_openrover_basic/charging",
            Bool,
            self.openrover_charging_cb,
            queue_size=1,
        )

        self.sub_undock = rospy.Subscriber(
            "/auto_dock/undock", Bool, self.undock_cb, queue_size=1
        )
        self.sub_cancel_auto_dock = rospy.Subscriber(
            "/auto_dock/cancel", Bool, self.cancel_cb, queue_size=1
        )
        self.sub_start = rospy.Subscriber(
            "/auto_dock/dock", Bool, self.start_cb, queue_size=1
        )

        # service for determining whether we're in range of the charging station
        rospy.wait_for_service("wibotic_connector_can/read_parameter")
        self.in_range_service = rospy.ServiceProxy(
            "wibotic_connector_can/read_parameter", ReadParameter
        )

        # Setup timers
        self.state_manager_timer = rospy.Timer(
            rospy.Duration(self.MANAGER_PERIOD), self.state_manage_cb, oneshot=False
        )
        # self.docking_timer = rospy.Timer(rospy.Duration(self.MAX_RUN_TIMEOUT), self.docking_failed_cb, oneshot=True)

    def state_manage_cb(self, event):
        # rospy.loginfo("%s | %s", self.docking_state, self.last_docking_state)
        if self.docking_state == "undock":
            self.disable_aruco_detections()
            self.undock_state_fun()

        if self.docking_state == "undocked":
            self.disable_aruco_detections()
            self.undocked_state_fun()
            # self.in_range_service_call() - don't think we need to be checking this here

        if self.docking_state == "searching":
            self.enable_aruco_detections()
            self.searching_state_fun()

        if self.docking_state == "centering":
            self.enable_aruco_detections()
            self.centering_state_fun()

        if self.docking_state == "approach":
            self.enable_aruco_detections()
            self.approach_state_fun()
            self.in_range_service_call()

        if self.docking_state == "final_approach":
            self.enable_aruco_detections()
            self.final_approach_state_fun()
            self.in_range_service_call()

        if self.docking_state == "wait_for_charge":
            self.disable_aruco_detections()
            self.wait_for_charge_state_fun()
            self.in_range_service_call()

        if self.docking_state == "docked":
            self.disable_aruco_detections()
            # self.in_range_service_call()  # not sure if we need to be checking this here

        if self.docking_state == "docking_failed":
            self.disable_aruco_detections()

        if self.docking_state == "cancelled":
            self.disable_aruco_detections()

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

    def searching_state_fun(self):
        rospy.loginfo("searching aruco count: %i", self.aruco_callback_counter)
        self.centering_counter = 0
        if self.action_state == "turning":
            return
        if self.aruco_callback_counter < self.ARUCO_CALLBACK_COUNTER_MAX:
            self.set_action_state("count_aruco_callbacks")
        else:
            self.aruco_callback_counter = 0
            self.set_action_state("")
            self.openrover_turn(-self.TURN_RADIANS)

    def centering_state_fun(self):
        # wait for another detection then center
        # rospy.loginfo('centering aruco count: %i', self.aruco_callback_counter)
        if self.action_state == "turning":
            return
        if self.aruco_callback_counter < 1:
            self.centering_counter = self.centering_counter + 1
            self.set_action_state("count_aruco_callbacks")
            return
        self.aruco_callback_counter = 0
        self.set_action_state("")
        if self.centering_counter >= self.MAX_CENTERING_COUNT:
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
                if abs(distance) < self.FINAL_APPROACH_DISTANCE:
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
                            (abs(distance) - self.FINAL_APPROACH_DISTANCE) / 2,
                            self.JOG_DISTANCE,
                        )
                    )
        else:
            # TODO/help - not sure what their logic was here -
            # the marker is not in view but go to final approach?
            self.openrover_stop()
            self.openrover_forward(self.FINAL_APPROACH_DISTANCE)
            self.set_docking_state("final_approach")

    def final_approach_state_fun(self):
        [theta, distance, theta_bounds] = self.fid2pos(self.dock_aruco_tf)
        if self.is_in_view and abs(distance) > self.FINAL_APPROACH_DISTANCE:
            self.openrover_stop()
            self.set_docking_state("approach")
            return
        if self.action_state == "jogging":
            return
        if self.action_state == "":
            # the linear timer for the forward command called in approach_state_fun
            # has timed out and the rover has stopped (presumably adjacent to charger)
            self.set_docking_state("wait_for_charge")

    def wait_for_charge_state_fun(self):
        # sometimes can take a couple seconds for the in_range or charge
        # confirmation to be received
        if self.action_state == "":
            rospy.loginfo("WAITING FOR CHARGE")
            self.set_action_state("waiting")
            self.waiting_timer = rospy.Timer(
                rospy.Duration(self.CHARGE_WAIT_TIME),
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
            self.openrover_forward(-self.UNDOCK_DISTANCE)
            self.undocking_state = "reversing"
            return
        if self.undocking_state == "reversing":
            rospy.logwarn("Undock turning")
            self.openrover_turn(3.0)
            self.is_undocked = True
            self.undocking_state = "turning"
            return
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
        self.is_charging = False
        self.is_in_range = False

        self.is_looking = False
        self.is_undocked = True
        self.is_in_action = False
        self.is_final_jog = False
        self.is_turning = False
        self.is_jogging = False
        self.is_undocking = False
        self.docking_failed = False

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
                abs(distance) / self.CMD_VEL_LINEAR_RATE * self.open_loop_correction
            )
            rospy.loginfo("jog_period: %f", jog_period)

            # this timer callback handles stopping at end of jog period
            # - sets bools is_in_action and is_jogging to False
            # - calls openrover_stop, which sets vel cmds to 0 and action state to ""
            self.linear_timer = rospy.Timer(
                rospy.Duration(jog_period), self.openrover_linear_timer_cb, oneshot=True
            )

            if distance > 0:
                rospy.loginfo("Moving forward")
                self.cmd_vel_linear = self.CMD_VEL_LINEAR_RATE
            else:
                rospy.loginfo("Moving Backward")
                # self.cmd_vel_linear = -self.CMD_VEL_LINEAR_RATE * 1.5
                self.cmd_vel_linear = -self.CMD_VEL_LINEAR_RATE
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
        z_trans = z_trans - self.Z_TRANS_OFFSET
        theta = math.atan2(x_trans, z_trans)
        r = math.sqrt(x_trans ** 2 + z_trans ** 2)
        # approach angle is some maximum amount of turning to center
        if r > self.APPROACH_RADIUS:
            theta_bounds = self.APPROACH_ANGLE
        else:
            theta_bounds = r / self.APPROACH_RADIUS * self.APPROACH_ANGLE
        # rospy.loginfo("z=%fm and x=%fm", z_trans, x_trans)
        # rospy.loginfo("Theta: %3.3f, r: %3.3f, x_trans: %3.3f, z_trans: %3.3f, x: %3.3f, y: %3.3f, z: %3.3f", theta, r, x_trans, z_trans, euler_angles[0], euler_angles[1], euler_angles[2])
        # rospy.loginfo(
        #     "Theta: %3.3f, r: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds
        # )
        # rospy.loginfo("distance = %s", r)
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
            turn_period = abs(radians / self.CMD_VEL_ANGULAR_RATE)
            if turn_period < self.MIN_TURN_PERIOD:
                turn_period = self.MIN_TURN_PERIOD

            # this timer callback handles stopping at end of turn period
            # - sets bools is_in_action and is_turning to False
            # - calls openrover_stop, which sets vel cmds to 0 and action state to ""
            self.turn_timer = rospy.Timer(
                rospy.Duration(turn_period), self.openrover_turn_timer_cb, oneshot=True
            )

            if radians > 0:
                rospy.loginfo("Turn right for %f", turn_period)
                self.cmd_vel_angular = -self.CMD_VEL_ANGULAR_RATE
            else:
                rospy.loginfo("Turn Left for %f", turn_period)
                self.cmd_vel_angular = self.CMD_VEL_ANGULAR_RATE
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
    def undock_cb(self, event):
        rospy.loginfo("undock_cb")
        if event.data == True and not self.docking_state == "cancelled":
            self.openrover_stop()
            self.full_reset()
            self.set_docking_state("undock")
            self.set_action_state("")

    def cancel_cb(self, event):
        rospy.loginfo("cancel_cb")
        if event.data:
            self.set_docking_state("cancelled")
            self.openrover_stop()
            self.full_reset()
            self.cancelled_timer = rospy.Timer(
                rospy.Duration(self.CANCELLED_TIMEOUT),
                self.cancelled_timer_cb,
                oneshot=True,
            )

    def cancelled_timer_cb(self, event):
        rospy.loginfo("cancelled_timer_cb")
        self.set_docking_state("undocked")

    def start_cb(self, event):
        rospy.loginfo("start_cb")
        self.openrover_stop()
        rospy.sleep(self.START_DELAY)
        if event.data and not (self.docking_state == "docked"):
            self.set_docking_state("searching")
            self.docking_timer = rospy.Timer(
                rospy.Duration(self.MAX_RUN_TIMEOUT),
                self.docking_failed_cb,
                oneshot=True,
            )

    def wait_now_cb(self, event):
        rospy.loginfo("wait_now_cb")
        if not (self.docking_state in ["docked", "wait_for_charge"]) and not (
            self.docking_state == "cancelled"
        ):
            self.set_docking_state("undocked")
            self.is_undocked = True
            self.openrover_stop()
            self.docking_timer.shutdown()

    def aruco_detect_cb(self, fid_tf_array):
        if not (self.docking_state in ["docked", "wait_for_charge"]) and not (
            self.docking_state == "cancelled"
        ):
            # handle timers related to timing out and warning if aruco is running slowly

            # ---this chunk is broken
            # aruco_now_time = rospy.Time.now()
            # aruco_cb_period = aruco_now_time-self.aruco_last_time
            # if aruco_now_time > (self.aruco_last_time+self.ARUCO_SLOW_WARN_TIMEOUT):
            #     rospy.logwarn("Aruco running at %2.3fHz.", aruco_cb_period.to_sec()/1000000000)
            # self.aruco_last_time = aruco_now_time
            # ---above chunk is broken

            # If no aruco cb's happen within ARUCO_WAIT_TIMEOUT seconds, then assume the image pipe has been disconnected and go into undocked state
            try:
                self.undocked_timer.shutdown()
                self.undocked_timer = rospy.Timer(
                    rospy.Duration(self.ARUCO_WAIT_TIMEOUT),
                    self.wait_now_cb,
                    oneshot=True,
                )
            except:
                self.undocked_timer = rospy.Timer(
                    rospy.Duration(self.ARUCO_WAIT_TIMEOUT),
                    self.wait_now_cb,
                    oneshot=True,
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
        self.is_turning = False
        self.is_in_action = False

    def openrover_linear_timer_cb(self, event):
        rospy.loginfo("openrover_linear_timer_cb: Stop moving forward")
        self.is_jogging = False
        self.is_in_action = False
        self.openrover_stop()

    def openrover_charging_cb(self, charging_msg):
        self.is_charging = charging_msg.data
        self.docked_fun()

    def in_range_service_call(self):
        try:
            data = self.in_range_service(name="CCHK").value
            # rospy.loginfo("wibotics service call gave: %s", data)
            if data == 1409286812:
                self.is_in_range = True
            self.docked_fun()
        except rospy.ServiceException as e:
            rospy.logwarn("error calling wibotic connector service: %s", e)

    def docked_fun(self):
        if self.docking_state == "undock":
            return
        if self.is_charging or self.is_in_range:
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
        rospy.loginfo("shutting down wait-for-charge timer")
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
