#!/usr/bin/env python

import rospy

from auto_dock import ArucoDockingManager


class MockedDockingManager(ArucoDockingManager):
    def init_wibotics_service_proxy(self):
        """overwriting method to mock wibotics service"""
        pass

    def in_antenna_range_service_call(self):
        self.docked_fun()

    def waiting_for_in_range_state_fun(self):
        """
        state entered after final approach
        mock immediate antenna detection
        everything else is the same as parent class
        """
        super(MockedDockingManager, self).waiting_for_in_range_state_fun()
        self.is_in_antenna_range = True


def auto_dock_main():
    docking_manager = MockedDockingManager()
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
