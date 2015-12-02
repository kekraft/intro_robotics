#!/usr/bin/env python

# Kory Kraft
# Basic Stopper Code
# BSD Licenses
# 10/21/2015

# up properly.  
import rospy
import math
import tf

# The laser scan message
from sensor_msgs.msg import LaserScan


# The velocity command message
from geometry_msgs.msg import Twist

class Stopper():
    def __init__(self, stop_dist, max_speed):
        self.STOP_DISTANCE = stop_dist
        self.MAX_SPEED = max_speed

        # publisher for moving
        self.velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # subscribing to laser scan so I can read it
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

    def laser_cb(self, scan):
        # Make a new Twist message
        command = Twist()

        # Fill in the fields.  Field values are unspecified until they are
        # actually assigned.  The Twist message holds linear and angular
        # velocities.
        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0

        # only look at +- 30 degrees in front of robot
        num_scans = len(scan.ranges)
        central_ranges = scan.ranges[(num_scans/4):-(num_scans/4)]
        min_dist = min(central_ranges)
        
        # if min_dist < STOP_DISTANCE:
        #     print "min dist ", min_dist
        #     # command.linear.x = 0.0
        #     # command.angular.x = 0.0
        #     print "Backing up"
        #     error = min_dist - STOP_DISTANCE 
        #     # command.linear.x = (SPEED * math.tanh(10.0 * error))
        #     # print command.linear.x
        #     command.linear.x = - .05
        # else:
        error = min_dist - self.STOP_DISTANCE
        command.linear.x = (self.MAX_SPEED * math.tanh(10.0 * error)) 
        if command.linear.x < -.01:
            command.linear.x = -.01

        # publish the command
        self.velocity_pub.publish(command)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('stopper', log_level=rospy.DEBUG) 
    
    if rospy.has_param('stop_dist'):
        STOP_DISTANCE = rospy.get_param('stop_dist')
    else:
        STOP_DISTANCE = 0.75

    if rospy.has_param('max_speed'):
        MAX_SPEED = rospy.get_param('max_speed')
    else:
        MAX_SPEED = 0.1

    stopper = Stopper(STOP_DISTANCE, MAX_SPEED) 
 
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     pub.publish(command)
    #     rate.sleep()
    rospy.spin()
