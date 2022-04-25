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

LINEAR_VEL = 0.1
STOP_DISTANCE = 0.13
LIDAR_ERROR = 0.05
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
        twist = Twist()
        turtlebot_moving = True
        angular_max_vel = 2.64 # max angular velocity in rad/s

        while not rospy.is_shutdown():
            
            # Get distances - center, left, right (30 samples each)
            lidar_distances = self.get_scan()
            lidar_left = lidar_distances[:31]
            lidar_center = lidar_distances[31:61]
            lidar_right = lidar_distances[61:]

            # min distance and angular velocity calculations
            min_distance = min(lidar_center)
            if (angular_max_vel > min_distance):
                angular_vel = abs(angular_max_vel - min_distance)
            else:
                angular_vel = 0

            # determine direction of rotation
            if(sum(lidar_left) < sum(lidar_right)):
                direction = -1
            else:
                direction = 1
            twist.angular.z = angular_vel*direction

            # stop if close to an obstacle
            if min_distance < STOP_DISTANCE and turtlebot_moving:
                twist.linear.x = 0
                turtlebot_moving = False
            else:
                twist.linear.x = LINEAR_VEL
                turtlebot_moving = True
            
            self._cmd_pub.publish(twist)

            




























            # Decide movement
            # MIN distance reached
            # if min_distance < SAFE_STOP_DISTANCE:
            #     # Stop robot
            #     if turtlebot_moving:
            #         twist.linear.x = 0.0
            #         twist.angular.z = 0.0
            #         self._cmd_pub.publish(twist)
            #         turtlebot_moving = False
            #         rospy.loginfo('Stop!')
                
            #     # Decide new direction
            #     else:
                    # Left is min - turn right
                    

                    # if min(lidar_distances[0:46]) < min(lidar_distances[46:92]):
                    #     twist.linear.x = 0.0
                    #     twist.angular.z = 0.3
                    #     self._cmd_pub.publish(twist)
                    #     rospy.loginfo('Turning right: Distance of the obstacle : %f', min(lidar_left))
                    
                    # # Right is min - turn left
                    # else:
                    #     twist.linear.x = 0.0
                    #     twist.angular.z = -0.3
                    #     self._cmd_pub.publish(twist)
                    #     rospy.loginfo('Turning left: Distance of the obstacle : %f', min(lidar_right))
            
            # MIN distance not reached
            # else:
            #     if min(lidar_left) <= min(lidar_center) and min(lidar_left) <= min(lidar_right):
            #         twist.linear.x = LINEAR_VEL
            #         twist.angular.z = 0.3
            #         self._cmd_pub.publish(twist)
            #         turtlebot_moving = True
            #         rospy.loginfo('Turning right: Distance of the obstacle : %f', min(lidar_left))
            #     if min(lidar_center) <= min(lidar_left) and min(lidar_center) <= min(lidar_right):
            #         twist.linear.x = LINEAR_VEL
            #         twist.angular.z = 0.0
            #         self._cmd_pub.publish(twist)
            #         turtlebot_moving = True
            #         rospy.loginfo('Going straight: Distance of the obstacle : %f', min(lidar_center))
            #     if min(lidar_right) <= min(lidar_center) and min(lidar_right) <= min(lidar_left):
            #         twist.linear.x = LINEAR_VEL
            #         twist.angular.z = -0.3
            #         self._cmd_pub.publish(twist)
            #         turtlebot_moving = True
            #         rospy.loginfo('Turning left: Distance of the obstacle : %f', min(lidar_right))


def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
