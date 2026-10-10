#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮径标定助手 —— 让车自己直线走一段，告诉你"里程计说走了多少"

【为什么需要】
  底盘固件里 WheelDiameter 是【写死的 64.0】✗ 轮胎磨损/气压/地面都会让它实际偏 ✗
  → 里程计就会系统性地多算或少算 → 走一段就停不准 ✗
  标定方法：让车走一段 → 问"实际走了多少" → 算出正确的轮径 ✓

【怎么用】（小车上，只需要 roscore + base_control）
    cd ~/chen_car_drone/小车 && source ~/catkin_ws/devel/setup.bash && python3 wheel_calib.py

  它会：
    1. 记录起点（/odom）
    2. 让车以 0.15 m/s 直线前进，使【里程计读数增加约 1.0 米】
    3. 停车，打印里程计说的距离
    4. ★ 然后你【用卷尺/地砖量一下车实际走了多远】，输入那个数
    5. 它立刻算出"正确的轮径应该是多少"（直接告诉你改成几）

【安全】默认速度 0.15 m/s、最多走 1.2 米就自动停；Ctrl+C 立即停
"""
import sys
import time
import math
import rospy
from geometry_msgs.msg import Twist

TARGET_M = 1.0          # 想让里程计读数增加 1.0 米
SPEED = 0.15            # 前进速度 m/s（慢一点，安全）
MAX_M = 1.2             # 硬上限：超过这个距离一定停
WHEEL_D_CFG = 64.0      # 当前配置里的轮径（launch 里 WheelDiameter: 64.0）


def main():
    rospy.init_node("wheel_calib", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    od = {"x": None, "y": None}

    def on_odom(msg):
        od["x"] = msg.pose.pose.position.x
        od["y"] = msg.pose.pose.position.y

    from nav_msgs.msg import Odometry
    rospy.Subscriber("/odom", Odometry, on_odom, queue_size=2)

    print("=" * 70)
    print(" 轮径标定助手   想让里程计读数增加 %.1f 米" % TARGET_M)
    print(" ★ 车前方留出【1.5 米以上】空地；随时 Ctrl+C 可停")
    print("=" * 70)
    t0 = time.time()
    while od["x"] is None and time.time() - t0 < 8:
        time.sleep(0.1)
    if od["x"] is None:
        print("✗ 收不到 /odom —— 检查 base_control 起来了吗")
        sys.exit(1)

    x0, y0 = od["x"], od["y"]
    print("起点里程计: x=%.4f y=%.4f" % (x0, y0))
    print("开始前进（%.2f m/s）…" % SPEED)

    tw = Twist()
    tw.linear.x = SPEED
    rate = rospy.Rate(20)
    last = (x0, y0)
    try:
        while not rospy.is_shutdown():
            pub.publish(tw)
            d = math.hypot(od["x"] - x0, od["y"] - y0)
            last = (od["x"], od["y"])
            if d >= TARGET_M or d >= MAX_M:
                break
            rate.sleep()
    except KeyboardInterrupt:
        print("\n（手动中断）")
    finally:
        tw.linear.x = 0.0
        for _ in range(10):                     # 多播几次确保停下
            pub.publish(tw)
            time.sleep(0.05)

    time.sleep(0.6)
    d_odom = math.hypot(last[0] - x0, last[1] - y0)
    dx, dy = last[0] - x0, last[1] - y0
    print()
    print("=" * 70)
    print(" 停车。里程计说走了： %.4f 米" % d_odom)
    print("   （起点 x=%.4f y=%.4f  →  终点 x=%.4f y=%.4f）" % (x0, y0, last[0], last[1]))
    print("=" * 70)
    print()
    print("★ 现在用卷尺 / 地砖量一下【车实际走了多远】，然后输入那个数（米）")
    print("   （不想量就直接回车跳过）")
    try:
        raw = input("实际距离(m) = ").strip()
    except (EOFError, KeyboardInterrupt):
        raw = ""
    if not raw:
        print("已跳过计算。实际距离 = 里程计距离 × (实际/里程计) 的比值就是校正系数")
        return
    try:
        d_real = float(raw)
    except ValueError:
        print("✗ 输入不是数字，跳过")
        return
    if d_odom <= 0.01 or d_real <= 0.01:
        print("✗ 距离太小，无法计算")
        return

    ratio = d_real / d_odom
    new_d = WHEEL_D_CFG * ratio
    print()
    print("=" * 70)
    print(" 结果")
    print("   里程计说: %.4f m   实际: %.4f m   比值: %.4f" % (d_odom, d_real, ratio))
    if abs(ratio - 1.0) < 0.01:
        print("   ✅ 已经很准（差 <1%%），轮径不用改 ✓")
    else:
        print("   ★ 轮径应该从 %.1f 改成 【%.2f】" % (WHEEL_D_CFG, new_d))
        print("     改哪： wd_base_control 的 launch 里 wheel_diameter 参数")
        print("     命令： grep -rn wheel_diameter $(rospack find wd_base_control)/launch/")
        print("     改完重启 base_control 生效 ✓")
    print("=" * 70)


if __name__ == "__main__":
    main()
