#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂单独调试 / 抓取距离测试 —— 不用起导航，只动机械臂

【为什么要它】
  要单独测"机械臂能夹到哪个位置"，不该每次都跑一遍导航+平台流程 ✗
  这个脚本让你手动发关节角、手动开合夹爪，并实时显示【码离相机多远】✓

【前提】
  roscore + base_control（TargetAngle 是 base_control 订阅的）✓
  看 AR 距离还需要：wd_vision bringup ✓

【怎么跑】（小车上）
    cd ~/chen_car_drone/小车 && source ~/catkin_ws/devel/setup.bash && python3 arm_jog.py

【命令】
  init / sentry / grap / up      直接切命名姿态
  close / open                   夹爪闭合(135) / 张开(30)
  j1 95     j2 -10    j3 225     单独调某个关节到某角度
  set 90 -13 230 -46 90 135      一次发 6 个值（最后一位 30=开 / 135=合）
  show                           打印当前关节反馈 + 姿态表
  ar                             单独查一次 AR 码距离
  help / quit
★ 每条命令执行后会自动报一次"码离相机多远"（等机械臂动完再读）
"""
import sys
import time
import rospy
from std_msgs.msg import String                                    # noqa: F401  (话题类型占位)
from sensor_msgs.msg import JointState
from ar_track_alvar_msgs.msg import AlvarMarkers

try:
    from wd_arm_moveit_demo.msg import ArmJoint
except ImportError:
    print("✗ 找不到 wd_arm_moveit_demo.msg.ArmJoint")
    print("  → 先跑：source ~/catkin_ws/devel/setup.bash")
    sys.exit(1)

# 和 mqtt_bridge_node.py 保持一致
POSES = {
    "init":   [90.0, -13.0, 230.0, -46.0, 90.0, 30.0],   # 折叠
    "sentry": [90.0,  90.0,  98.0, -40.0, 90.0, 30.0],   # 抬起看货（相机朝下）
    "grap":   [90.0,  47.0,   0.0,  91.0, 88.0, 30.0],   # 伸下去抓
    "up":     [90.0,  90.0,  90.0,  90.0, 90.0, 30.0],   # 全中位
}
GRIP_OPEN, GRIP_CLOSE = 30.0, 135.0
RUN_TIME = 2000                                   # 毫秒
SAFE = [(0, 180), (-20, 200), (-20, 270), (-100, 180), (-20, 270), (-20, 200)]

cur = list(POSES["init"])                         # 当前目标角
js = {"pos": None}
ar = {"d": None, "id": None, "t": 0.0}


def on_js(msg):
    try:
        js["pos"] = list(msg.position)
    except Exception:
        pass


def on_ar(msg):
    if msg.markers:
        p = msg.markers[0].pose.pose.position
        ar["d"] = (p.x * p.x + p.y * p.y + p.z * p.z) ** 0.5
        ar["id"] = int(msg.markers[0].id)
        ar["t"] = time.time()


def main():
    rospy.init_node("arm_jog", anonymous=True)
    pub = rospy.Publisher("/TargetAngle", ArmJoint, queue_size=10)
    rospy.Subscriber("/joint_states", JointState, on_js, queue_size=1)
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, on_ar, queue_size=5)
    time.sleep(0.8)

    def ar_line():
        if ar["d"] is not None and time.time() - ar["t"] < 2.0:
            return "看到 ★%s 号，距离 %.3f m" % (ar["id"], ar["d"])
        return "没看到码"

    def send(wait=True):
        for i, (v, (lo, hi)) in enumerate(zip(cur, SAFE)):
            if not (lo <= v <= hi):
                print("  ✗ j%d=%s 超范围 %s~%s（会被底盘静默丢弃，未发送）" % (i + 1, v, lo, hi))
                return False
        m = ArmJoint()
        m.joints = [float(x) for x in cur]
        m.run_time = RUN_TIME
        pub.publish(m)
        print("  → 已发 %s（%.1f 秒）" % ([int(x) for x in cur], RUN_TIME / 1000.0))
        if wait:
            time.sleep(RUN_TIME / 1000.0 + 0.4)     # 等机械臂动完再看码
            print("     %s" % ar_line())
        return True

    def show():
        print("  目标值:", [int(x) for x in cur])
        if js["pos"]:
            print("  反馈弧度:", ["%.3f" % v for v in js["pos"]])
        else:
            print("  反馈: 没收到 /joint_states ✗（base_control 起了吗？）")
        print("  AR:", ar_line())

    print("=" * 74)
    print(" 机械臂单独调试   输 help 看命令")
    print(" 姿态表:", {k: [int(x) for x in v] for k, v in POSES.items()})
    print(" 夹爪 open=30 / close=135   安全范围 j1 0~180 j2 -20~200 j3 -20~270 j4 -100~180 j5 -20~270 j6 -20~200")
    print("=" * 74)

    while not rospy.is_shutdown():
        try:
            line = input("arm> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        p = line.split()
        cmd = p[0].lower()

        if cmd in ("quit", "exit", "q"):
            break
        elif cmd == "help":
            print("  init/sentry/grap/up | close/open | j1..j6 <角度> | set <6个数> | show | ar | quit")
        elif cmd in POSES:
            cur[:] = list(POSES[cmd]); print("  切到", cmd); send()
        elif cmd == "close":
            cur[5] = GRIP_CLOSE; send()
        elif cmd == "open":
            cur[5] = GRIP_OPEN; send()
        elif cmd in ("j1", "j2", "j3", "j4", "j5", "j6") and len(p) >= 2:
            cur[int(cmd[1]) - 1] = float(p[1]); send()
        elif cmd == "set" and len(p) >= 7:
            cur[:] = [float(x) for x in p[1:7]]; send()
        elif cmd == "show":
            show()
        elif cmd == "ar":
            print("  AR:", ar_line())
        else:
            print("  ? 不认识的命令，输 help")


if __name__ == "__main__":
    main()
