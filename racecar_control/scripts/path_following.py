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
        self.goal_tolerance = 3
        self.orientation = 0
        self.position = np.array([0, 0, 0])
        self.goal = np.array([15, 2, 0])
        self.start_pos = None
        self.u_turn = False
        self.reached_pos = False
        self.done = False

        # U turn variables
        self.going_forward = False
        self.going_back = False
        self.ang_goal = 0
        self.uturn_speed = 0.7
        self.uturn_steering = 0.37
        self.obs_distance = 1

    def go_forward(self, msg):
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

    def check_obs_uturn(self, msg):
        l2 = len(msg.ranges)/2;
        ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]

        obstacle_detected = False
        if self.going_forward:
            for i in range(l2-l2/8, l2+l2/8) :
                if np.isfinite(ranges[i]) and ranges[i]>0 and ranges[i] < self.obs_distance:
                    obstacle_detected = True
                    break

        elif self.going_back:
                for i in range(l2-l2/8, l2+l2/8) :
                    if np.isfinite(msg.ranges[i]) and msg.ranges[i]>0 and msg.ranges[i] < self.obs_distance:
                        obstacle_detected = True
                        break

        return obstacle_detected

    def do_u_turn(self, msg):
        if not self.going_back and not self.going_forward:
            self.going_forward = True
            self.ang_goal = self.orientation + np.pi
            if self.ang_goal > np.pi:
                self.ang_goal = -np.pi + (self.ang_goal % np.pi)
            self.cmd_vel_pub.publish(Twist())
            rospy.loginfo("Started U turn, orientation: " + str(self.orientation))

        elif self.going_forward:
            cmd = Twist()
            cmd.linear.x = self.uturn_speed
            cmd.angular.z = self.uturn_steering
            self.cmd_vel_pub.publish(cmd)

        elif self.going_back:
            cmd = Twist()
            cmd.linear.x = -self.uturn_speed
            cmd.angular.z = -self.uturn_steering
            self.cmd_vel_pub.publish(cmd)

        if self.check_obs_uturn(msg):
            self.going_back = not self.going_back
            self.going_forward = not self.going_forward

        if np.abs(self.ang_goal - self.orientation) < 0.1:
            print(self.ang_goal)
            print(self.orientation)
            self.u_turn = False
            rospy.loginfo("Finished U turn, orientation: " + str(self.orientation))

    def reached_goal(self, goal):
        reached_goal = np.linalg.norm(self.position - goal) < self.goal_tolerance
        if reached_goal:
            rospy.loginfo("Reached goal:" + str(self.position))
        return reached_goal

    def scan_callback(self, msg):

        if self.u_turn and not self.done:
            self.do_u_turn(msg)

        elif not self.reached_pos and not self.done:
            self.go_forward(msg)
            rospy.loginfo("Goal:" + str(self.goal))
            rospy.loginfo("Position: " + str(self.position))
            if self.reached_goal(self.goal):
                self.reached_pos = True
                self.u_turn = True

        elif self.reached_pos and not self.done:
            self.go_forward(msg)
            rospy.loginfo("Goal:" + str(self.start_pos))
            rospy.loginfo("Position: " + str(self.position))
            if self.reached_goal(self.start_pos):
                self.done = True

        else:
            self.cmd_vel_pub.publish(Twist())
            pass
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.position = np.array([x, y, z])
        self.orientation = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.start_pos is None:
            self.start_pos = self.position
       
        # rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)

def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

