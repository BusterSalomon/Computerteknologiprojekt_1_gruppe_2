#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.3
LIDAR_ERROR = 0.05
TURNING_DISTANCE = 1.7
TURNING_CONSTANT = 1.1
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
ANGULAR_MAX_VEL = 2.64 # max angular velocity in rad/s

class Obstacle():
    # Attributes
    collisions = 0
    
    # Methods

    # ----- INIT ------
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
    
    # ----- GET SCAN -----
    # Returns an array of 91 lidar scans
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 91           # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    # ------ GET DIRECTION ------
    # Pre: Takes left average, center average and right average as parameters
    # Post: Returns direction: 0, 1 or -1
    # -1: Right
    # 0: Forward
    # 1: Left
    def get_direction (self, left_avrg, center_avrg, right_avrg):
        direction = 0
        if (center_avrg < TURNING_DISTANCE):
            if (abs(left_avrg - right_avrg) < 0.1 and center_avrg > SAFE_STOP_DISTANCE): # need explanation
                direction = 0
                rospy.loginfo("going STRAIGHT!")
            elif(left_avrg > right_avrg ):
                direction = -1
                rospy.loginfo("turning RIGHT!")
            else:
                direction = 1
                rospy.loginfo("turning LEFT!")
            rospy.loginfo("left avrg: %f", left_avrg)
            rospy.loginfo("right avrg: %f", right_avrg)
        return direction

    # ----- SPLIT SCAN -----
    # Pre: Takes an array of lidar scans
    # Post: Returns 3 arrays with lidar scans, left, center and right
    def split_scan(self, lidar_distances):
        lidar_left = [d for d in lidar_distances[10:36] if d != 0]
        lidar_center = [d for d in lidar_distances[35:56] if d != 0]
        lidar_right = [d for d in lidar_distances[55:81] if d != 0]
        return (lidar_left, lidar_center, lidar_right)
    
    # ------ GET CENTER AVERAGE -----
    # Pre: Takes the center array scans as argument
    # Post: Splits the center array into two, takes the average of both, and returns the smallest average
    # Purpose: ...
    def get_center_avrg(self, center_arr):
        center_size = len(center_arr)
        if center_size == 1: return center_arr[0]
        center_avrg = min([sum(center_arr[:(center_size/2)]) / (center_size/2), sum(center_arr[(center_size/2):]) / (center_size/2)])
        return center_avrg

    # ----- GET ANGULAR VELOCITY -----
    # Pre: Takes the center average distance
    # Post: Calculates and returns the angular velocity.
    # The angular velocity is calculated so it increases as it moves closer to obstacles
    # NOTEE: we could make a graph/function that shows how much is turns based on the center average! Maybe we should make it exponentiel instead of linear?
    def get_angular_vel(self, center_avrg):
        if (center_avrg < TURNING_DISTANCE):
            angular_vel = abs(ANGULAR_MAX_VEL - (center_avrg * 1.4)) * TURNING_CONSTANT # !!!SHOULD BE REFACTORED!!!
        else: 
            angular_vel = 0
        return angular_vel
    
    # ----- GET LINEAR VELOCITY ----- 
    # Pre: Takes center average, angular velocity and direction as parameters
    # Post: Returns the linear velocity, and a collision flag
    # The linear velocity is calculated so it goes slower as it moves closer to an obstacle
    # NOTEE: math function that shows the linear speed as a function of center avrg!
    def get_linear_vel(self, center_avrg, direction):
        linear_vel = 0
        collision_flag = False
        if center_avrg < SAFE_STOP_DISTANCE:
            linear_vel = -0.1
            collision_flag = True
            rospy.loginfo('MOVING SLIGHTLY BACKWARDS')
        else:
            linear_vel = (LINEAR_VEL - (angular_vel / (ANGULAR_MAX_VEL*5)) / 3)*direction
        return linear_vel, collision_flag
    
    # ---- CONDTIONAL UPDATE COLLISIONS COUNTER -----
    # Pre: Takes a collision flag as parameter
    # Post: Updates the global collision attrubute - doesn't return
    def cond_update_collisions (self, collision_flag):
        if (collision_flag): 
            self.collisions += 1


    # ----- OBSTACLE: HIGH LEVEL -----
    # Highest level method
    # Does the following in order:
    # 1. Get distances
    # 2. split distances into tree parts; left, center and right
    # 3. get average of the distances
    # 4. get direction based on the average distances
    # 5. get angular velocity based on the center average distance
    # 6. get linear velocity based on the center average distance
    # 7. updates collision counter
    # 8. Publish the velocities
    def obstacle(self):
        twist = Twist()
        rospy.loginfo("Obstacle Avoidance Starts")

        while not rospy.is_shutdown():
            # LOG - loop is started
            rospy.loginfo('loop start_______________')
            
            # Get distances array and split scans
            lidar_distances = self.get_scan()
            lidar_left, lidar_center, lidar_right = self.split_scan(lidar_distances)
            
            # If any scans, continue - otherwise, scan again
            if (lidar_center and lidar_left and lidar_right): 
                
                # Get averages
                center_avrg = self.get_center_avrg(lidar_center)
                left_avrg = sum(lidar_left) / len(lidar_left)
                right_avrg = sum(lidar_right) / len(lidar_right)

                # LOG center average
                rospy.loginfo("center_avrg: %f", center_avrg)
                
                # Default set direction to straight
                direction = self.get_direction(left_avrg, center_avrg, right_avrg)

                # Get velocities and collision flag
                angular_vel = self.get_angular_vel(center_avrg)
                linear_vel, collision_flag = self.get_linear_vel(center_avrg, direction)
                
                # Conditional update collisions counter
                self.cond_update_collisions(collision_flag)

                # Apply movement
                twist.linear.x = linear_vel
                twist.angular.z = angular_vel
                self._cmd_pub.publish(twist)


    
                

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
