#!/usr/bin/env python
''' 
# Kory Kraft
# Returns turtlebot to its "home" when the 'h' key is pressed
# BSD Licenses
# 10/21/2015
'''


# Every python controller needs these lines
import rospy
import actionlib

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import *

import sys, select, termios, tty

class Return():

    def __init__(self, home_x=2.0, home_y=2.0):
        self.home_x = home_x
        self.home_y = home_y

        # rospy.on_shutdown(self.shutdown_node)

    def check_input(self):
        while(1):
            # try:
                key = self.get_key()

                if key == 'h':
                    print "return home!"

                    self.send_move_goal(30.0)

                    print "returned home, checking keys again"
            # except:
            #     # print "Checking input failed"
            #     print "Unexpected error:", sys.exc_info()[0]
            #     pass

    def get_key(self):
        try:

            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:                
                key = sys.stdin.read(1)

            else:
                key = ''

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

            # print "Got key ", key
            return key

        except:
            print "Couldn't get key"
            print "Unexpected error:", sys.exc_info()[0]
            return 


    def send_move_goal(self, wait_time):
        
        # create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'

        if rospy.has_param('~home_x'):
            self.home_x = rospy.get_param('~home_x')
        else:
            self.home_x = 2.07

        if rospy.has_param('~home_y'):
            self.home_y = rospy.get_param('~home_y')
        else:
            self.home_y = 2.11

        goal.target_pose.pose.position.x = self.home_x
        goal.target_pose.pose.position.y = self.home_y
        goal.target_pose.pose.position.z = 0.00
        goal.target_pose.pose.orientation.x = 0.00
        goal.target_pose.pose.orientation.y = 0.00
        goal.target_pose.pose.orientation.z = 0.988
        goal.target_pose.pose.orientation.w = 0.156

        goal.target_pose.header.stamp = rospy.Time.now()

        # create the action client and send goal
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server(rospy.Duration.from_sec(wait_time))

        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(60.0))
        # client.send_goal_and_wait(goal, wait_time)

        print client.get_result()
        print client.get_state()

        return client



if __name__ == '__main__':
    ## For keyboard input
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('task_manager') 

    print "\n\n\n\n\nTo return the robot to its home, press the \'h\' key.\n\n\nSet the home_x and home_y params using rosparam set.\n\n\n\n"

    # if rospy.has_param('~home_x'):
    #     home_x = rospy.get_param('~home_x')
    # else:
    #     home_x = 2.07

    # if rospy.has_param('~home_y'):
    #     home_y = rospy.get_param('~home_y')
    # else:
    #     home_y = 2.11

    returner = Return()
    returner.check_input()
 
    #rospy.spin()

    ## to terminate keyboard input (without this, program won't stop)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)