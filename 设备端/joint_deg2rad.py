#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
关节状态单位转换节点
====================
背景：
  下位机驱动 wd_base_control 发布的关节状态是【度】（舵机角度制，如 -17.584），
  而 MoveIt / URDF / robot_state_publisher 要求【弧度】（如 -0.3069）。
  → MoveIt 拿到 -17.584 当弧度用 → 远超关节限位 ±1.5708 → "invalid start state" → 规划失败

方案（不改任何 C++ 代码）：
  ① base_control.launch 里把驱动节点的话题 remap 成 joint_states_deg
  ② 本节点订阅 joint_states_deg（度）→ 乘 π/180 → 发布 /joint_states（弧度）
  ③ MoveIt / robot_state_publisher 照常读 /joint_states，拿到的就是弧度 ✓

★ 回退：停掉本节点 + 删掉 base_control.launch 里那行 remap 即可。
"""
import math
import rospy
from sensor_msgs.msg import JointState

IN_TOPIC = "/joint_states_deg"      # 输入：下位机发的（度）
OUT_TOPIC = "/joint_states"         # 输出：转成弧度（MoveIt 读这个）
DEG2RAD = math.pi / 180.0

pub = None

def cb(msg):
    """收到度数 → 每个关节角度乘 π/180 → 发出去"""
    out = JointState()
    out.header = msg.header
    out.name = msg.name
    out.position = [p * DEG2RAD for p in msg.position]   # ★ 度 → 弧度
    out.velocity = msg.velocity
    out.effort = msg.effort
    pub.publish(out)

def main():
    global pub
    rospy.init_node("joint_deg2rad", anonymous=True)
    pub = rospy.Publisher(OUT_TOPIC, JointState, queue_size=10)
    rospy.Subscriber(IN_TOPIC, JointState, cb, queue_size=10)
    rospy.loginfo("[joint_deg2rad] 已启动：%s (度)  -->  %s (弧度)", IN_TOPIC, OUT_TOPIC)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
