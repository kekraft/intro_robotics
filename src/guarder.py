#!/usr/bin/env python

# Kory Kraft
# Guards teleoperation of the turtlebot.
# BSD Licenses
# 10/21/2015

import rospy
import math
import tf
import message_filters

# The laser scan message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


# The velocity command message
from geometry_msgs.msg import Twist, TwistStamped

class Guard():
    def __init__(self, stop_dist, max_speed):
        self.STOP_DISTANCE = stop_dist
        self.MAX_SPEED = max_speed

        # publisher for moving
        self.velocity_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.teleop_stamped_pub = rospy.Publisher('cmd_vel_mux/input_unfiltered_stamped/teleop', TwistStamped, queue_size=10)

        self.scan_sub = message_filters.Subscriber('/scan', LaserScan)
        self.teleop_sub = message_filters.Subscriber('cmd_vel_mux/input_unfiltered/teleop', Twist)
        self.teleop_stamped_sub = message_filters.Subscriber('cmd_vel_mux/input_unfiltered_stamped/teleop', TwistStamped)

        # give a timestamp to twist message
        self.teleop_sub.registerCallback(self.twist_cb)

        self.teleop_n_scan_sub = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.teleop_stamped_sub], 1, 100)
        self.teleop_n_scan_sub.registerCallback(self.teleop_n_scan_cb)

    def teleop_n_scan_cb(self, scan, twist_stamped):
        # get teleop twist message
        command = twist_stamped.twist

        # only look at +- 30 degrees in front of robot
        num_scans = len(scan.ranges)
        central_ranges = scan.ranges[(num_scans/4):-(num_scans/4)]
        min_dist = min(central_ranges)

        ## let this be the driving error....
        if not command.linear.x == 0.00:
            error = min_dist - self.STOP_DISTANCE
            command.linear.x = (self.MAX_SPEED * math.tanh(10.0 * error)) 
            if command.linear.x < 0.00:
                command.linear.x = 0.00 # don't allow driving backwards, too dangerous! ;)

        # publish the command
        self.velocity_pub.publish(command)

    def twist_cb(self, twist):
        # get teleop twist message
        twist_stamped = TwistStamped()
        twist_stamped.twist = twist

        if not (twist.linear.x == 0 and twist.linear.y == 0 and twist.linear.z == 0 and twist.angular.x == 0 and twist.angular.y == 0 and twist.angular.z == 0):          

            # give it a header
            header = Header()
            header.stamp = rospy.Time.now()
            twist_stamped.header = header

            # publish the command
            self.teleop_stamped_pub.publish(twist_stamped)


if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('guard') 
    
    if rospy.has_param('stop_dist'):
        STOP_DISTANCE = rospy.get_param('stop_dist')
    else:
        STOP_DISTANCE = 0.75

    if rospy.has_param('max_speed'):
        MAX_SPEED = rospy.get_param('max_speed')
    else:
        MAX_SPEED = 0.1

    guard = Guard(STOP_DISTANCE, MAX_SPEED) 
 
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     pub.publish(command)
    #     rate.sleep()
    rospy.spin()
