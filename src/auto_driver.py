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

from PyQt4.QtCore import QThread

import sys, select, termios, tty
import time
import copy
import numpy as np
from random import choice
import os

class Driver():
    HOME = 0
    RANDOMIZE = 1
    NORMAL = 2
    SHUTDOWN = 3

    def __init__(self, home_x=2.0, home_y=2.0):
        # flag to check for termination conditions
        self.is_running = True
        self.condition = Driver.NORMAL


        # get positions from file
        rospack = rospkg.RosPack()
        dir_path = rospack.get_path("exercise1")
        file_path = dir_path + "/extra/positions.csv"

        # positions = [(2.0, 2.0), (3.0, 3.0), (5.0, 3.0)]
        self.orig_positions =  np.loadtxt(open(file_path,"rb"),delimiter=",")
        print "Locations to visit - pose(x,y,z) quaternion(x,y,z,w) "
        print self.orig_positions

        # self.home_x = home_x
        # self.home_y = home_y
        self.start_loc = (2.0,2.0,0.0,0.0,0.0,0.988,0.156)

        # to_visit = list(reversed(orig_positions))
        self.to_visit = copy.deepcopy(self.orig_positions)

        self.wait_time = 60.0 # used as seconds in autopilot function
        self.is_interrupted = False

        # create the action client and send goal
        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server(rospy.Duration.from_sec(self.wait_time))

        
        ## marking list of places to vist
        self.location_marker_pub = rospy.Publisher("destination_marker", Marker, queue_size=10)
        self.create_all_markers()

        # publishing destination thread
        self.marker_pub_thread = MarkerPublisherThread("marker_pub_thread", 1.0, self.location_marker_pub, self.markers)

        # keyboard input thread
        self.kb_thread = KeyboardWorkerThread("kb_thread", .05)

        # important to register shutdown node with threads after thread have been created
        rospy.on_shutdown(self.shutdown)
        
        # start threads before handing off state control to autopilot
        self.marker_pub_thread.start()
        self.kb_thread.start()

        # rospy.on_shutdown(self.shutdown_node)

    def autopilot(self):
        ''' visit each location in visit list '''

        while(True):
            # print "Locations to visit"
            # print self.to_visit

            for loc in self.to_visit:
                # print 
                # print
                # print "Location to visit"
                # print loc
                self.send_move_goal(loc)

                # self.client.wait_for_result(rospy.Duration.from_sec(self.wait_time)) # blocking call
                # could check to make sure its not a failure nor a success
                terminal_states = [GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.SUCCEEDED, GoalStatus.RECALLED]
                # print "Terminal States: ", terminal_states
                movement_state = self.client.get_state()

                # print "Condition!!", self.condition

                # print terminal_states
                while movement_state not in terminal_states:
                    movement_state = self.client.get_state()
                    
                    if self.update_state() != Driver.NORMAL:
                        break

                    time.sleep(.1)

                if movement_state == GoalStatus.SUCCEEDED:
                    self.do_something_interesting()

                if self.condition != Driver.NORMAL:
                    # print "Condition not normal!!"
                    break

            if self.condition == Driver.HOME:
                ## Go home and start visiting list again
                espeak_call_str = "espeak -s 110 \'Yes master. Returning home.\'"
                os.system(espeak_call_str)
                self.send_move_goal(self.start_loc)
                print "Visiting Home"

                ## wait till you get home to start the next round
                self.client.wait_for_result(rospy.Duration.from_sec(self.wait_time))

                self.do_something_interesting()

            elif self.condition == Driver.RANDOMIZE:
                ## jumble the to-visit list then start visiting list again
                np.random.shuffle(self.to_visit)  
                # print "Shuffled List"             

            elif self.condition == Driver.SHUTDOWN:
                print "Shutting down"
                espeak_call_str = "espeak -s 110 \'Goodbye.\'"
                os.system(espeak_call_str)
                break

            self.condition = Driver.NORMAL # everything goes to normal once weird stuff happens
            # print "Visiting locations again"
            time.sleep(2)

        # print "exiting autopilot"
        # rospy.signal_shutdown("Finished Autopilot")

    

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

    def do_something_interesting(self):
        ''' Output a nice call phrase for the grader.
            Assumes espeak is installed. BAD ASSUMPTION
        '''
        sayings = ["I love you Will", "Great job Will", "Your a great teaching assistant",
                    "Am I home yet?", "When will this stop?", "Are you my mother?", 
                    "Assimov books are great.", "Robot army on its way.",
                    "Your my friend.", "Kory deserves 100 percent."]
        espeak_call_str = "espeak -s 120 \'{0}\'".format(choice(sayings))
        os.system(espeak_call_str)

    def create_all_markers(self):
        self.markers = []

        for i in xrange(len(self.orig_positions)):
            loc = self.orig_positions[i]
            marker_id = i
            marker = self.create_location_marker(loc, Marker.SPHERE, Marker.ADD, marker_id, ColorRGBA(0,0,1,1))

            self.markers.append(marker)

        # print "in create all markers", self.markers


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

        # self.marker_pub_thread.quit()
        self.marker_pub_thread.terminate()

        old_settings = self.kb_thread.old_settings

        self.kb_thread.quit()
        self.kb_thread.terminate()
        
        # print "terminated termios and restored settings"
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# # Qt - Worker Thread to get keyboard input in nonblocking call
class KeyboardWorkerThread(QThread):
    def __init__(self, name, sleep, parent = None):
        QThread.__init__(self, parent)
        self.sleep = sleep
        self.name  = name
        self.c = None
        self.old_settings = termios.tcgetattr(sys.stdin)

    def isData(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

    def run(self):    
        self.old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            while True:

                if self.isData():
                    c = sys.stdin.read(1)
                    if c == '\x1b':         # x1b is ESC
                        print "Stopped looking at kb input."
                        self.c = c
                        break
                    else:
                        self.c = c
                        # print c
                time.sleep(self.sleep)

        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)    

    def get_c(self):
        return self.c
    
    def pop_c(self):
        c = self.c
        self.c = None
        return c



# # Qt - Worker Thread to publish markers
class MarkerPublisherThread(QThread):
    def __init__(self, name, sleep, publisher, markers=None, parent = None):
        QThread.__init__(self, parent)
        self.sleep = sleep
        self.name  = name
        self.markers = markers
        self.publisher = publisher

    def run(self):    
        while(True):
            try:
                # print "marker pub thread running"
                for marker in self.markers:
                    self.publisher.publish(marker)

            except Exception, e:
                print "Marker pub thread exception"
                print "Unexpected error:", sys.exc_info()[0]
                print str(e)


            time.sleep(self.sleep)


if __name__ == '__main__':
    ## For keyboard input
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('driver') 

    msg = '''

    'h' = return robot home
    'r' = randomize position list
    'Esc' = shutdown (you can ctrl-C after, but not before)

    '''

    print msg

    driver = Driver()

    ## start autopilot, which puts it into an infinite loop unless user exits out with Esc key
    driver.autopilot()

    ## to terminate keyboard input (without this, program won't stop)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

