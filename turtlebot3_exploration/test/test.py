#!/usr/bin/env python3
PKG = 'turtlebot_exploration'

import sys
import unittest
import rospy
import rostest

PKG='turtlebot3_exploration'

from turtlebot3_exploration.planner import Planner

class TestPlanner(unittest.TestCase):
    
    def test_arriving_at_goal(self):
        """ Test if the robot arrives at the goal.
        """
        
        from actionlib_msgs.msg import GoalStatus
        rospy.init_node('test_arriving_at_goal', log_level=rospy.DEBUG)
        p = Planner()
        p.go_to_goal(1, 0.75, 0)
        rospy.print("Goal is active")
        while p.status != GoalStatus.SUCCEEDED:
            p.rate.sleep()
        self.assertEqual(p.status, GoalStatus.SUCCEEDED)

if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_planner', TestPlanner)
    