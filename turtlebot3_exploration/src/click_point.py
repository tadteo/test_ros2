#!/usr/bin/env python3

import rospy
from turtlebot3_exploration.planner import Planner
from geometry_msgs.msg import Pose, Point, Quaternion

def main():
    """ The main() function """
    rospy.init_node('planner_example', log_level=rospy.DEBUG)

    p = Planner()
    p.planner_loop()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except:
        rospy.ROSInterruptException


