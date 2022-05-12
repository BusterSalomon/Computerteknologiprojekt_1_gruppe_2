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

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.3
LIDAR_ERROR = 0.05
TURNING_DISTANCE = 1.5
TURNING_CONSTANT = 1.1
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
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

    def obstacle(self):
        rospy.loginfo("Obstacle Avoidance Starts")
        twist = Twist()
        turtlebot_moving = True
        angular_max_vel = 2.64 # max angular velocity in rad/s

        while not rospy.is_shutdown():
            rospy.loginfo('loop start_______________')
            # Get distances - center, left, right
            lidar_distances = self.get_scan()
            lidar_left = [d for d in lidar_distances[:45] if d != 0]
            lidar_center = [d for d in lidar_distances[35:56] if d != 0]
            lidar_right = [d for d in lidar_distances[46:] if d != 0]
            if (lidar_center and lidar_left and lidar_right):
                center_size = len(lidar_center)
                if center_size == 1: center_size = 2

                center_avrg = min([sum(lidar_center[:(center_size/2)]) / (center_size/2), sum(lidar_center[(center_size/2):]) / (center_size/2)])
                
                # min distance and angular velocity calculations
                rospy.loginfo("center_avrg: %f", center_avrg)
                direction = 0
                if (center_avrg < TURNING_DISTANCE):
                    angular_vel = abs(angular_max_vel - center_avrg) * TURNING_CONSTANT
                    # determine direction of rotation
                    if(sum(lidar_left) < sum(lidar_right)):
                        direction = 1
                    else:
                        direction = -1
                    rospy.loginfo("sum left: %f", min(lidar_left))
                    rospy.loginfo("sum right: %f", min(lidar_right))
                else:
                    angular_vel = 0

                rospy.loginfo("angular_vel: %f", angular_vel*direction)
                twist.angular.z = angular_vel*direction

                # stop if close to an obstacle
                if center_avrg < SAFE_STOP_DISTANCE and turtlebot_moving:
                    twist.linear.x = 0
                    turtlebot_moving = False
                    rospy.loginfo('STOP!!!!!!!!!!!!')
                else:
                    twist.linear.x = LINEAR_VEL
                    turtlebot_moving = True
                
                self._cmd_pub.publish(twist)
                rospy.loginfo('publishing msg!____________')

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
