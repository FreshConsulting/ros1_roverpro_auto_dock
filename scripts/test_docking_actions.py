#!/usr/bin/env python

import rospy
import actionlib

import rr_auto_dock.msg


class TestDockingClient:
    def __init__(self):
        rospy.loginfo("launching test client node")
        self._dock_client = actionlib.SimpleActionClient(
            "/auto_dock/dock", rr_auto_dock.msg.AutoDockAction
        )
        self._undock_client = actionlib.SimpleActionClient(
            "/auto_dock/undock", rr_auto_dock.msg.AutoDockAction
        )
        self.dock_goal = rr_auto_dock.msg.AutoDockGoal()
        self.undock_goal = rr_auto_dock.msg.AutoDockGoal()

        rospy.loginfo("Test Client waiting for action servers...")
        self._dock_client.wait_for_server()
        self._undock_client.wait_for_server()

        self.docking_state = ""

    def dock(self):
        self.docking_state = ""
        rospy.loginfo("sending docking goal")
        self._dock_client.send_goal(self.dock_goal, feedback_cb=self.dock_feedback_cb)

        time_spent = 0
        while not self.docking_state == "searching" and time_spent < 8:
            rospy.sleep(1)
            time_spent += 1

        return self.docking_state

    def dock_feedback_cb(self, feedback):
        self.docking_state = feedback.docking_state

    def cancel_dock(self):
        rospy.loginfo("cancelling docking goal")
        self._dock_client.cancel_goal()
        self._dock_client.wait_for_result()
        return self._dock_client.get_result()

    def undock(self):
        rospy.loginfo("Sending undock goal")
        self._undock_client.send_goal(self.undock_goal)
        rospy.loginfo("Client waiting for undock result")
        self._undock_client.wait_for_result()
        return self._undock_client.get_result()


def test_undock(client):
    rospy.loginfo("testing undock action")
    assert client.undock().complete == True


def test_dock(client):
    rospy.loginfo("testing docking action")
    # currently just testing that it transitions into the searching state
    # because we'd need to add more mocks to complete the docking (such as aruco detect)
    assert client.dock() == "searching"


def test_preempt(client):
    rospy.loginfo("testing preempt dock action")
    assert client.cancel_dock().complete == False


if __name__ == "__main__":
    try:
        rospy.init_node("test_docking_actions_client")
        client = TestDockingClient()
        test_undock(client)
        test_dock(client)
        test_preempt(client)
        test_undock(client)
        test_dock(client)
        test_preempt(client)

    except rospy.ROSInterruptException:
        rospy.loginfo("test client program interrupted before completion")
