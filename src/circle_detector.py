#!/usr/bin/env python
'''

'''

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from math import sin,cos,atan2,pi,sqrt
import matplotlib.pyplot as plt

## for ellipse fitting
import numpy as np
from ellipse2d import Ellipse2d
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors

class CircleDetector:
    def __init__(self, max_size=1.0, min_size=0.01, axis_a=0.9, center_a=0.1, laser_topic="laser"):
        self.max_size = max_size
        self.min_size = min_size
        self.axis_alpha = axis_a
        self.center_alpha = center_a
        self.last_a = None
        self.last_b = None
        self.last_center = None
        # self.red = ColorRGBA(1, 0, 0, 1)
        # self.green = ColorRGBA(0, 1, 0, 1)
        # self.color = self.red

        self.scan_sub = rospy.Subscriber(laser_topic, LaserScan, self.laser_cb)
        self.marker_array_pub = rospy.Publisher("circle_marker_array", MarkerArray, queue_size=10)

        print "Instantiated circle detector"



    def laser_cb(self, data):
        # Fit ellipse
        # If ellipses, create and place markers

        # self.fit_ellipse(data) # single ellipse expected
        ellipses = self.fit_multi_ellipse(data) # multiple ellipses expected

        # create markers
        markers = []
        if ellipses is not None:
            for ellipse in ellipses:
                print "Found ellipse"
                marker = self.create_circle_marker(ellipse.center[0], ellipse.center[1], ellipse.theta, ellipse.a, ellipse.b)
                markers.append(marker)

        # place markers
        self.marker_array_pub.publish(markers)


    # Fit an ellipse to the data
    # Assumes there is just one ellipse in the data
    def fit_ellipse(self, data):
        #print "fitting"
        ellipse_xy = []
        #points = [] #array to hold all points - FOR DEBUGGING

        angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges
        #polar >> cartesian

        for r in ranges:
            if r < max_range:
                ellipse_xy.append([cos(angle)*r, sin(angle)*r]) #make xy
            angle += incr

        if len(ellipse_xy) > 1:
            
            ellipse = Ellipse2d()
            ellipse.fit(ellipse_xy)

            if self.is_valid_ellipse(ellipse): 
                #apply alpha to smooth changes over time, if old data exists
                if self.last_a != None and self.last_b != None and self.last_center != None:
                    ellipse.center = [self.last_center[i]*self.center_alpha + ellipse.center[i]*(1-self.center_alpha) for i in [0, 1]]
                    ellipse.a = self.last_a*self.axis_alpha + ellipse.a*(1-self.axis_alpha)
                    ellipse.b = self.last_b*self.axis_alpha + ellipse.b*(1-self.axis_alpha)
                
                self.last_center = ellipse.center
                self.last_b = ellipse.b
                self.last_a = ellipse.a


    def fit_multi_ellipse(self, data):
        ## Fits multiple ellipses to laser scan 
        ## using clustering algorithm and fit ellipse code
        ## Does check to see if all ellipses are valid according to 
        ## meta parameters for circular objects

        angle = data.angle_min
        incr = data.angle_increment
        max_range = data.range_max
        ranges = data.ranges
        points = []
        
        for r in ranges:
            #add all valid ranges to some xy range
            if r < max_range:
                points.append([cos(angle)*r, sin(angle)*r])
            angle += incr

        #eps = range, min_samples = min# of points in cluster.
        points = np.asarray(points)
        if len(points) > 3:
            db = DBSCAN(eps=0.5, min_samples=3).fit(points)
        else:
            return None

        #core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
        #core_samples_mask[db.core_sample_indices_] = True
        labels = db.labels_
        #return points, labels
        
        # Number of clusters in labels, ignoring noise if present.
        n_clusters = len(set(labels)) - (1 if -1 in db.labels_ else 0)
        new_ellipses = []
        for n in xrange(n_clusters):
            xy = points[labels==n]
            e = Ellipse2d()
            try:
                e.fit(xy)
                print e
                if self.is_valid_ellipse(e):
                    print "  valid ellipse"
                    new_ellipses.append(e)
            except:
                pass

        print "Number ellipses found: ", len(new_ellipses)
        return new_ellipses
            
        
    def is_valid_ellipse(self, ellipse):
        # Validity is measured by it being a real ellipse, with 
        #   values in the plausible range for representing a human
        # if (ellipse.is_valid() and 
        #     (self.min_size < ellipse.a < self.max_size) and 
        #     (min_size < ellipse.b < max_size)):
        #     return True
        # else:
        #     return False

        if ellipse.is_valid():
            return True
        else:
            return False


    def create_circle_marker(self, pose_x, pose_y, ellipse_theta, ellipse_a, ellipse_b):
        h = Header()
        # h.frame_id = self.scan_frame_id #tie marker visualization to laser it comes from
        h.frame_id = "map"
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        
        #create marker:person_marker, modify a red cylinder, last indefinitely
        mark = Marker()
        mark.header = h
        mark.ns = "ellipse_marker"
        mark.id = 0
        mark.type = 3
        mark.action = 0
        mark.scale = Vector3(ellipse_a*2, ellipse_b*2, 1) #scale, in meters
        mark.color = ColorRGBA(0, 1, 0, 1) 

        pose = Pose(Point(pose_x, pose_y, 0.5), Quaternion(0.0,0.0,1.0,cos(ellipse_theta/2)))
        mark.pose = pose

        return mark


if __name__ == '__main__':
    rospy.init_node("circle_detector", anonymous=False)

    max_size = 1.2 #rospy.get_param("~max_size")
    min_size = 0.2 #rospy.get_param("~min_size")
    axis_alpha = 1.0 #rospy.get_param("~axis_alpha")
    axis_center = 1.0 #rospy.get_param("~axis_center")

    detector = CircleDetector(max_size, min_size, axis_alpha, axis_center, "/scan")
    
    rospy.spin()