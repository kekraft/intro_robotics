#!/usr/bin/env python
''' 
# Kory Kraft
# Visits a list of locations from the positions.csv file in the extra folder.
# Starts a thread for keyboard input.
# Starts a thread for publishing location markers.
# Returns turtlebot to its "home" when the 'h' key is pressed
# Randomizes list when 'r' is pressed.
# Exits out when Esc pressed.
# 
# The program still does not shutdown correctly with ctrl-C (nned to signal shutdown with signal module, I believe...)
#
# BSD Licenses
# 11/18/2015
'''


# Every python controller needs these lines
import rospy
# import actionlib
from actionlib import SimpleActionClient, GoalStatus
import rospkg

from geometry_msgs.msg import Twist
from std_msgs.msg import String, ColorRGBA, Header
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from nav_msgs.msg import OccupancyGrid, MapMetaData

import copy
import numpy as np
from random import choice
import os

class Explorer():
    EXPLORING = 0
    DONE = 1

    def __init__(self, home_x=2.0, home_y=2.0):
        # flag to check for termination conditions
        self.is_running = True
        self.condition = Driver.NORMAL

        # initialized the "viewed "
        self.map = OccupancyGrid()

        # subscribe to the view_map
        self.map_sub = rospy.Subscriber("view_map", OccupancyGrid, self.update_map)

        # get file path
        # rospack = rospkg.RosPack()
        # dir_path = rospack.get_path("exercise1")

        # self.home_x = home_x
        # self.home_y = home_y

        # to_visit = list(reversed(orig_positions))
        self.to_visit = copy.deepcopy(self.orig_positions)

        self.wait_time = 60.0 # used as seconds in autopilot function
        self.is_interrupted = False

        # create the action client and send goal
        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server(rospy.Duration.from_sec(self.wait_time))
        
        ## marking list of places to vist
        self.location_marker_pub = rospy.Publisher("destination_marker", Marker, queue_size=10)
        # self.create_all_markers()

        # important to register shutdown node with threads after thread have been created
        rospy.on_shutdown(self.shutdown)


    def 

    def explore(self):

        ## plant flags

        ## don't stop until you visit everything

   

    def update_state(self):
        char = self.kb_thread.pop_c()

        if char == 'h':
            self.condition = Driver.HOME
        elif char == 'r':
            self.condition = Driver.RANDOMIZE
        elif char == '\x1b':
            self.condition = Driver.SHUTDOWN
            # print "shutdown condition"
        else:
            # self.condition = Driver.NORMAL
            pass

        return self.condition

    def cancel_goal(self):
        self.client.cancel_all_goals()

    def send_move_goal(self, loc):
        
        # create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'

        goal.target_pose.pose.position.x = loc[0]
        goal.target_pose.pose.position.y = loc[1]
        goal.target_pose.pose.position.z = loc[2]
        goal.target_pose.pose.orientation.x = loc[3]
        goal.target_pose.pose.orientation.y = loc[4]
        goal.target_pose.pose.orientation.z = loc[5]
        goal.target_pose.pose.orientation.w = loc[6]

        goal.target_pose.header.stamp = rospy.Time.now()

        self.client.send_goal(goal)

        return self.client

    # def do_something_interesting(self):
    #     ''' Output a nice call phrase for the grader.
    #         Assumes espeak is installed. BAD ASSUMPTION
    #     '''
    #     sayings = ["I love you Will", "Great job Will", "Your a great teaching assistant",
    #                 "Am I home yet?", "When will this stop?", "Are you my mother?", 
    #                 "Assimov books are great.", "Robot army on its way.",
    #                 "Your my friend.", "Kory deserves 100 percent."]
    #     espeak_call_str = "espeak -s 120 \'{0}\'".format(choice(sayings))
    #     os.system(espeak_call_str)

    # def create_all_markers(self):
    #     self.markers = []

    #     for i in xrange(len(self.orig_positions)):
    #         loc = self.orig_positions[i]
    #         marker_id = i
    #         marker = self.create_location_marker(loc, Marker.SPHERE, Marker.ADD, marker_id, ColorRGBA(0,0,1,1))

    #         self.markers.append(marker)

    #     # print "in create all markers", self.markers


    def mark_as_to_visit(self, loc, marker_id):
        marker = self.create_location_marker(loc, Marker.SPHERE, Marker.ADD, marker_id, ColorRGBA(0,1,0,1))
        self.location_marker_pub.publish(marker)

    def mark_as_visited(self, loc, marker_id):
        marker = self.create_location_marker(loc, color, Marker.SPHERE, Marker.ADD, marker_id, ColorRGBA(0,1,1,1))
        self.location_marker_pub.publish(marker)

    def create_location_marker(self, loc, marker_type=Marker.SPHERE, marker_action=Marker.ADD, marker_id=0, marker_color=ColorRGBA(1,0,0,1)):
        h = Header()
        h.frame_id = "map"
        h.stamp = rospy.Time.now()
        
       
        mark = Marker()
        mark.header = h
        mark.ns = "location_marker"
        mark.id = marker_id
        mark.type = marker_type
        mark.action = marker_action
        mark.scale = Vector3(0.25, 0.25, 0.25) 
        mark.color = marker_color 

        pose = Pose(Point(loc[0], loc[1], loc[2]), Quaternion(loc[3],loc[4],loc[5],loc[6]))
        mark.pose = pose

        return mark

    ### threaded way to do it
    def shutdown(self):
        self.client.cancel_all_goals()




if __name__ == '__main__':
    rospy.init_node('explorer') 

    msg = '''

    Exploring map.

    '''

    print msg

    explorer = Explorer()
    explorer.explore()

    rospy.spin()

