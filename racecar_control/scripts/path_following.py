#!/usr/bin/env python

import rospy
import math 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from libcontrol import *


class PathFollowing:
    def __init__(self):
        self.max_speed = rospy.get_param('~max_speed', 0.75)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.position = np.array([0, 0, 0])
        self.goal = np.array([1, 1, 1])

    def scan_callback(self, msg):

        #if dist == numpy.linalg.norm(self.goal - self.position) < 0.5:

        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        l2 = len(msg.ranges)/2;
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        nb_fliter_pass = 1

        # Smooth ranges
        for i in range(nb_fliter_pass):
            for i, p in enumerate(ranges[(len(ranges)//4):(len(ranges)*3//4)]):
                idx = i + len(ranges)//4
                p = 0.4 * p + 0.2 * (ranges[idx+1] + ranges[idx-1]) + 0.1 * (ranges[idx+2] + ranges[idx-2])

        # Use only front values
        ranges = ranges[len(ranges)//4 : len(ranges)*3//4]


        # Find furthest direction
        dir_idx = ranges.index(np.amax(np.asarray(ranges)[np.asarray(ranges) != np.inf]))
        steering_dir = idx_to_steering(dir_idx, len(ranges), self.max_steering)
        

        # Find closest obstacle
        dir_idx = ranges.index(min(ranges))
        steering_obs = -idx_to_steering_inv(dir_idx, len(ranges), self.max_steering)

        steering = 0.6* steering_dir + 0.4 * steering_obs
        
        twist = Twist()
        twist.linear.x = self.max_speed
        twist.angular.z = steering
            
        self.cmd_vel_pub.publish(twist)

    # else:
    #     self.cmd_vel_pub.publish(Twist())

        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.position = np.array([x, y, z])
        #rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)

def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

