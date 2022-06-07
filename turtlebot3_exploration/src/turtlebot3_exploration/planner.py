#!/usr/bin/env python3
"""
 The planner node causes the robot to explore the environment autonomously while mapping the world
 SUBSCRIBERS:
  sub_map (nav_msgs/OccupancyGrid) - represents a 2-D grid map, in which each cell represents the probability of occupancy.
"""

import rospy
import actionlib
import numpy as np
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

from random import randrange
import time

PKG='turtlebot3_exploration'
class Planner:

    def __init__(self):
        """ Initialize environment
        """
        
        # Initialize rate:
        self.rate = rospy.Rate(1)

        # Move Base Action Client:
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5.0))
        rospy.loginfo("move_base is ready") 

        # Initialize subscribers:
        self.map = OccupancyGrid()
        self.odom = Odometry()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatus, self.goal_status_callback)
        
        self.goal_point = None
        self.goal = None
        self.status_list = []
        self.base_feedback = None
                
    def planner_loop(self):
        """ Main loop for planner.
        """
        
        while not rospy.is_shutdown():
                        
            # rospy.loginfo(f"odom: {self.odom.child_frame_id, self.odom.pose.pose.position.x, self.odom.pose.pose.position.y}")
            rospy.logdebug(f"State: {self.status_list}")
            rospy.sleep(2)
    
    def goal_status_callback(self, data):
        """ Check the status of a goal - goal reached, aborted,
        or rejected.
        """
        
        self.status_list = data.status_list
    
    def odom_callback(self, data):
        self.odom = data
        
    def map_callback(self, data):
        """ Callback function for map subscriber.
        """
        valid = False

        while valid is False:
            map_size = randrange(len(data.data))
            self.map = data.data[map_size]

            edges = self.check_neighbors(data, map_size)
            if self.map != -1 and self.map <= 0.2 and edges is True:
                valid = True

    def base_feedback_callback(self, data):
        """ Callback function for base feedback subscriber.
        """
        self.base_feedback = data
        # rospy.logdebug("Feedback: " + str(data))

    def go_to_goal(self,x=None,y=None):
        """ Set goal position for move_base.
        """
        rospy.loginfo("Setting goal")

        # Create goal:
        goal = MoveBaseGoal()

        # Set random goal:
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        
        rospy.loginfo(f"goal: {goal}")
        
        self.move_base.send_goal(goal)
        return goal

    def check_neighbors(self, data, map_size):
        """ Checks neighbors for random points on the map.
        """
        unknowns = 0
        obstacles = 0

        for x in range(-3, 4):
            for y in range(-3, 4):
                row = x * 384 + y
                try:
                    if data.data[map_size + row] == -1:
                        unknowns += 1
                    elif data.data[map_size + row] > 0.65:
                        obstacles += 1
                except IndexError:
                    pass
        if unknowns > 0 and obstacles < 2:
            return True
        else:
            return False
