#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KDL 逆解测试 —— 验证"给一个末端坐标，能不能算出关节角"

【为什么写它】
  既有实现注释里说"本机没装逆解插件"造成失败。
  实测：插件在（libmoveit_kdl_kinematics_plugin.so）✓ move_group 能起 ✓
  → 那问题就只剩 kinematics.yaml 里两个参数：timeout 5ms + 容差 0.1mm ✗
  这个脚本直接问 move_group 要逆解，用数据说话 ✓

【前提】
  ★ 另一个终端已经跑着 move_group：
      cd ~/catkin_ws && source devel/setup.bash && roslaunch arm_moveit_config demo.launch
    （demo.launch 会顺带起 robot_state_publisher → 机械臂 TF 链也建起来了 ✓
      RViz 崩了没关系 ✗ 它不影响 move_group ✓）

【怎么用】
    cd ~/chen_car_drone/小车 && source ~/catkin_ws/devel/setup.bash && python3 test_ik.py
  ★ 它不会动机械臂 ✗ 只做【规划(inverse kinematics + plan)】→ 纯计算 ✓ 安全 ✓

【看什么】
  · 成功率：20 次里算出来几次
  · 耗时：每次多少毫秒
  ★ 现在（timeout=5ms）大概率失败 ✗ → 改完参数再跑应成功 ✓
"""
import sys
import time

import rospy
import moveit_commander
from geometry_msgs.msg import Pose

GROUP = "arm_group"      # 规划组名（kinematics.yaml 里就是 arm_group）
N = 20                   # 试多少次


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("test_ik", anonymous=True)

    print("=" * 74)
    print(" KDL 逆解测试   规划组 = %s" % GROUP)
    print(" 只做【计算/规划】，不动机械臂 ✓")
    print("=" * 74)

    try:
        g = moveit_commander.MoveGroupCommander(GROUP)
    except Exception as e:
        print("✗ 连不上规划组 %s：%s" % (GROUP, e))
        print("  → move_group 起来了吗？组名对吗？（可用 rosrun moveit_ros_planning_interface list_move_group_parsers）")
        return

    g.set_planning_time(1.0)          # 每次最多让规划器想 1 秒（默认 5 秒太慢）
    g.allow_replanning(True)

    ee = ""
    try:
        ee = g.get_end_effector_link()
    except Exception:
        pass
    print("末端 link   :", ee or "(没设置)")
    try:
        joints = g.get_joints()
        cur = g.get_current_joint_values()
        print("关节数量    :", len(joints))
        print("当前关节角  :", [round(v, 4) for v in cur])
    except Exception as e:
        print("读当前关节失败:", e)
    print()

    # 当前末端位姿作为基准
    try:
        base_pose = g.get_current_pose().pose
    except Exception as e:
        print("✗ 读不到当前末端位姿：%s" % e)
        return
    print("当前末端位姿: x=%.4f y=%.4f z=%.4f" % (base_pose.position.x, base_pose.position.y, base_pose.position.z))
    print()
    print("-" * 74)
    print(" 开始测试：在当前位姿附近随机偏移 1~3cm，看能不能算出解并规划过去")
    print("-" * 74)

    ok = 0
    times = []
    import random
    for i in range(N):
        p = Pose()
        p.orientation = base_pose.orientation
        p.position.x = base_pose.position.x + random.uniform(-0.03, 0.03)
        p.position.y = base_pose.position.y + random.uniform(-0.03, 0.03)
        p.position.z = base_pose.position.z + random.uniform(0.0, 0.02)
        g.set_pose_target(p)
        t0 = time.time()
        try:
            res = g.plan()            # Noetic 返回 tuple (success, traj, time, error_code)
            if isinstance(res, tuple):
                good = bool(res[0])
                ec = res[3] if len(res) > 3 else None
                msg = "err=%s" % (getattr(ec, "val", ec) if ec is not None else "-")
            else:                     # 老版本可能直接返回 trajectory
                good, msg = bool(res), "trajectory"
        except Exception as e:
            good, msg = False, "异常:%s" % type(e).__name__
        dt = (time.time() - t0) * 1000.0
        times.append(dt)
        ok += 1 if good else 0
        print("  #%02d  %s  %6.1f ms   %s" % (i + 1, "✅ 成功" if good else "❌ 失败", dt, msg))
        g.clear_pose_targets()

    print()
    print("=" * 74)
    print(" 结果： %d / %d 成功   平均 %.1f ms" % (ok, N, sum(times) / len(times) if times else 0))
    print("=" * 74)
    if ok == 0:
        print(" ★ 全失败 → 印证了【参数问题】✗")
        print("   下一步：改 arm_moveit_config/config/kinematics.yaml")
        print("     kinematics_solver_timeout: 0.005   → 0.05")
        print("     goal_position_tolerance:   0.0001  → 0.005")
        print("     goal_orientation_tolerance: 0.001  → 0.05")
        print("   改完【重启 move_group】再跑本脚本 ✓")
    elif ok < N:
        print(" ★ 部分成功 → 参数还是偏紧，建议按上面的值放宽，再测一次 ✓")
    else:
        print(" ★★ 全成功 → 逆解【可用】✓✓ 可以进入下一步：解注释 + 重编译 ar_pick_place.cpp")
    print("\n★ 注意：本脚本只做计算 ✓ 机械臂不会动 ✓")


if __name__ == "__main__":
    main()
