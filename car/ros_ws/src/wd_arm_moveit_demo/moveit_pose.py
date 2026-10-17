#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
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

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy
from std_msgs.msg import UInt8,String

import math
import numpy

class NanoArmPickPlaceDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_pose', anonymous=False)
        
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = MoveGroupCommander('arm_group')
        
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'arm_base_link'
        self.arm.set_pose_reference_frame('arm_base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.01)
        
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(1.0)
        self.arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        self.target_pose = PoseStamped()
        target_pose_sub = rospy.Subscriber("target_pose",PoseStamped,self.callback)
        rospy.Subscriber("target_named",String,self.named_callback)
        
        
        rospy.spin()
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def callback(self,data):
        self.target_pose.header.frame_id = self.reference_frame
        self.target_pose.header.stamp = rospy.Time.now()     
        self.target_pose.pose = data.pose
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(self.target_pose, self.end_effector_link)
        self.arm.go()

    def named_callback(self,data):
        print(data.data)
        self.arm.set_named_target(data.data)
        self.arm.go()

if __name__ == "__main__":
    try:
        NanoArmPickPlaceDemo()
    except rospy.ROSInterruptException:
        pass
