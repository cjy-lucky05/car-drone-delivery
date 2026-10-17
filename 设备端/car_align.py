#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_align.py —— 视觉对准（画面内微调）
★ v3：判据修正 —— 前后用【z（距离）】控制，不是 y。
      （实测数据：物块在镜头正前方时 x≈0、y≈-6mm，但 z=0.41m 太远 → 机械臂够不到）
      识别逻辑照抄 mqtt_bridge_node.py：【只在有码时更新 + 窗口内看到过就算】。

作用：车停下后，用相机里物块的位置做闭环，把车挪到【和示教时一样】的状态，再抓。
      不需要相机标定、不需要 TF、不需要手眼标定。

用法：
  0) 持续观察（★ 先跑这个）：
       python3 car_align.py --watch 30
  1) 示教（把车手动挪到【能夹住】的位置后跑）：
       python3 car_align.py --teach
     ★ 记下的 z 就是目标 z（关键！）
  2) 只算不动：
       python3 car_align.py --tx 0.0016 --tz 0.25 --arm 0 --dry
  3) 真跑：
       python3 car_align.py --tx 0.0016 --tz 0.25 --arm 0

参数：
  --id       marker id（默认 3）
  --tx/--tz  目标值（覆盖下面的常量）
  --seen     多久内看到过就算看到（秒，默认 2.0）
  --wait     单次读码最多等多久（秒，默认 5.0）
  --tol_x    横向容差 m（默认 0.008 = 8mm）
  --tol_z    距离容差 m（默认 0.020 = 2cm）★ 太紧车会一直来回蹭，别小于 1.5cm
  --step/--settle  每步走多久 / 走完停多久（默认 0.25 / 0.50）
  --dir_x/--dir_z  方向符号（默认 -1 / +1；试一次不对就改）
  --max      最多多少步（默认 20）
  --speed_z/--speed_x  速度（默认 0.08 rad/s / 0.03 m/s）
  --arm      1=对准前发一次 sentry（默认 1）；你的方案直发 TargetAngle → 用 0

★ 安全：低速；每步走完必停再判断；有步数上限；结束/中断都发零速。
"""

import argparse
import math
import sys
import threading
import time

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from ar_track_alvar_msgs.msg import AlvarMarkers
except ImportError:
    AlvarMarkers = None

# ══════════ 示教得到的目标值（用 --teach 读出来填这里） ══════════
TARGET_X = None      # ← 横向（例：0.0083）  用 --teach 读的 x
TARGET_Y = None      # ← 上下（例：0.0092）  ★ 控前后就靠它
TARGET_Z = None      # ← 距离（例：0.2133）  仅参考/备用
# ═══════════════════════════════════════════════════════════════


class Aligner(object):
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        # ★★ 照 bridge 写法：只记【最后一次有效】读数 + 时间戳
        self.last_xy = None
        self.last_ts = 0.0
        self.seen_total = 0
        self.msg_total = 0

        if AlvarMarkers is None:
            print("✗ 找不到 ar_track_alvar_msgs —— 先 source ~/catkin_ws/devel/setup.bash")
            sys.exit(1)

        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.on_markers, queue_size=1)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_named = rospy.Publisher("/target_named", String, queue_size=1)
        self.pub_cancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

    # ---------- 回调：★ 只在有码时更新 ----------
    def on_markers(self, msg):
        self.msg_total += 1
        if not msg.markers:
            return                       # ★★ 空消息不动它，不覆盖上一次 ✓
        hit = None
        for m in msg.markers:
            if m.id == self.a.id:
                hit = m
                break
        if hit is None:
            return
        p = hit.pose.pose.position
        with self.lock:
            self.last_xy = (p.x, p.y, p.z)
            self.last_ts = time.time()
            self.seen_total += 1

    def read_xyz(self, wait=None):
        """★ 判据：窗口内【看到过】就算看到（不是"这一帧有"）"""
        wait = self.a.wait if wait is None else wait
        t0 = time.time()
        while time.time() - t0 < wait:
            with self.lock:
                xy, ts = self.last_xy, self.last_ts
            if xy is not None and (time.time() - ts) <= self.a.seen:
                return xy
            time.sleep(0.1)
        return None

    # ---------- 停车 / 准备 ----------
    def stop(self):
        # 停车也连发几次，确保底盘收到（发一次可能丢）
        for _ in range(5):
            self.pub_cmd.publish(Twist())
            time.sleep(0.02)

    def prepare(self):
        print("【准备】取消导航目标（防 move_base 抢控制）…")
        self.pub_cancel.publish(GoalID())
        time.sleep(1.0)
        self.stop()
        if self.a.arm:
            print("【准备】发一次 sentry …")
            self.pub_named.publish(String(data="sentry"))
            time.sleep(2.5)
        else:
            print("【准备】不动机械臂（--arm 0）—— 请确保它已在 sentry ✓")

    # ---------- 持续观察 ----------
    def watch(self, secs):
        print("═" * 64)
        print("持续观察 %d 秒（统计识别率）—— 请保持物块在画面里" % secs)
        print("═" * 64)
        t0 = time.time()
        while time.time() - t0 < secs:
            time.sleep(1.0)
            with self.lock:
                xy, ts, st, mt = self.last_xy, self.last_ts, self.seen_total, self.msg_total
            left = int(secs - (time.time() - t0))
            if xy is not None and (time.time() - ts) <= 2.0:
                print("  剩%3ds   ✓ 刚看到  x=%+.4f y=%+.4f z=%.4f   (有码 %d/%d 帧)"
                      % (left, xy[0], xy[1], xy[2], st, mt))
            else:
                ago = ("%.1fs 前" % (time.time() - ts)) if ts else "从没看到"
                print("  剩%3ds   ✗ 现在没有（%s）   (有码 %d/%d 帧)" % (left, ago, st, mt))
        with self.lock:
            st, mt = self.seen_total, self.msg_total
        print("─" * 64)
        print("★ 结果：%d 秒内收到 %d 帧，其中【有码 %d 帧】(%.0f%%)"
              % (secs, mt, st, (100.0 * st / mt) if mt else 0.0))
        if st == 0:
            print("   ✗ 一帧都没认到 → 查：物块躺下没 / 太远没 / 过曝没 / 码太小没")
        elif st < mt * 0.3:
            print("   ⚠ 断续 → 对准能用 ✓ 但会慢；★ 把车往前挪近一点识别率会明显上去 ✓")
        else:
            print("   ✓ 识别稳定 → 可以跑对准 ✓")
        if st:
            with self.lock:
                pass
        return 0

    # ---------- 主循环 ----------
    def run(self):
        a = self.a
        tx = a.tx if a.tx is not None else TARGET_X
        ty = a.ty if a.ty is not None else TARGET_Y
        tz = a.tz if a.tz is not None else TARGET_Z
        if tx is None or ty is None:
            print("✗ 还没有目标值：先跑  python3 car_align.py --teach")
            print("  把读出来的 x 填 TARGET_X、y 填 TARGET_Y（或用 --tx/--ty 传）")
            return 1

        print("═" * 64)
        print("视觉对准   目标 x=%+.4f（横向）  y=%+.4f（上下→控前后）   (marker id=%d)"
              % (tx, ty, a.id))
        print("           容差 x=%.1fmm  y=%.1fmm     最多 %d 步"
              % (a.tol_x * 1000, a.tol_y * 1000, a.max))
        if a.dry:
            print("           ★ DRY：只算不动（不发 cmd_vel）")
        print("═" * 64)

        self.prepare()
        miss = 0

        for step in range(1, a.max + 1):
            if rospy.is_shutdown():
                break

            cur = self.read_xyz()
            if cur is None:
                miss += 1
                with self.lock:
                    st, mt = self.seen_total, self.msg_total
                print("  第%-2d步   ✗ 这轮没看到（连续 %d 次；总帧 %d/%d 有码）" % (step, miss, st, mt))
                if miss >= 3:
                    print("\n✗ 连续 3 轮看不到 → 先跑 `--watch 30` 看真实识别率")
                    self.stop()
                    return 2
                continue
            miss = 0

            x, y, z = cur
            dx, dy = x - tx, y - ty
            ok_x, ok_y = abs(dx) <= a.tol_x, abs(dy) <= a.tol_y
            print("  第%-2d步   x=%+.4f y=%+.4f z=%.4f   Δx=%+6.1fmm Δy=%+6.1fmm   %s%s"
                  % (step, x, y, z, dx * 1000, dy * 1000,
                     "✓" if ok_x else "✗", "✓" if ok_y else "✗"))

            if ok_x and ok_y:
                print("\n✓ 对准成功（%d 步）：Δx=%.1fmm Δy=%.1fmm" % (step, dx * 1000, dy * 1000))
                self.stop()
                return 0

            tw = Twist()
            # ★★★★★ 横向【只能靠转向修】——实测"纯后退"几乎不改横向：
            #   相机朝下看，后退只是平移，物块横向角 = atan(横向/距离)，
            #   距离 0.22→0.25 时角只变 1°（实测 Δx 39.3→38.3mm，16 步动 1mm）。
            #   而【车头转过的角度 ≈ 物块在画面里横移的角度（1:1）】，
            #   所以横向修正就靠转向，但增益必须够大（kx 默认 8），
            #   否则 Δx 一小、w 就小，最后卡在十几毫米收不动。
            #   后退/前进只负责把"距离"调到能夹的位置。
            # ★★ 死区：某个方向已经够准了就不动它
            if abs(dy) <= a.tol_y:
                v = 0.0
            else:
                v = a.dir_z * max(-a.vmax, min(a.vmax, -a.ky * dy))
            if abs(dx) <= a.tol_x:
                w = 0.0
            else:
                w = a.dir_x * max(-a.wmax, min(a.wmax, a.kx * dx))

            # ★★★★★ 阿克曼曲率约束：底盘会拒绝曲率 > 1/min_turning_radius 的
            #   指令 —— 表现是【前轮打了、后轮不走、车纹丝不动】。
            #   实测踩坑：按 v = |w|/kappa_max 算出来的车速是【刚好卡在边界】，
            #   底盘因为浮点/取整判定超限 → 拒绝执行 → 车不动（读数一模一样）。
            #   所以留 15% 余量：v = |w| / kappa_max × 1.15
            kappa_max = 1.0 / a.min_radius
            if abs(w) > 1e-6:
                need_v = abs(w) / kappa_max * 1.15
                if abs(v) < need_v:
                    # ★ v=0（前后已够准）时也要给个方向，否则转不了；
                    #   方向取"让 Δy 远离 0 的那一侧"最自然，都没有就前进。
                    if v == 0.0:
                        sgn = 1.0 if (dy <= 0.0 or a.dir_z >= 0) else -1.0
                    else:
                        sgn = math.copysign(1.0, v)
                    v = sgn * need_v
            mode = "弧线"
            tw.linear.x = v
            tw.angular.z = w
            what = "%s  前后=%+.3f 转向=%+.3f  (Δy=%+.0fmm, Δx=%+.0fmm)" % (
                mode, v, w, dy * 1000, dx * 1000)

            if a.dry:
                print("        [dry] 本该发：%s（走 %.2fs）" % (what, a.step))
            else:
                # ★★★★★ 关键：底盘要【持续收到】cmd_vel 才走 ——
                #   单独 publish 一次可能不执行（实测：rostopic pub -r 20 能动，
                #   publish 一次不动）。所以在这 step 秒内按 20Hz 连续发。
                print("        → %s（走 %.2fs，20Hz 连发）" % (what, a.step))
                t0 = time.time()
                while time.time() - t0 < a.step:
                    self.pub_cmd.publish(tw)
                    time.sleep(0.05)
                self.stop()
                time.sleep(a.settle)

        print("\n✗ 到步数上限还没对准 → 结束（别硬抓）")
        self.stop()
        return 3

    # ---------- 示教 ----------
    def teach(self):
        print("═" * 64)
        print("示教：把车挪到【机械臂能夹住物块】的位置，然后读 x / z")
        print("═" * 64)
        xy = self.read_xyz(wait=15)
        if xy is None:
            print("✗ 15 秒内没看到码 → 先跑：python3 car_align.py --watch 30")
            return 1
        x, y, z = xy
        print("\n★ 物块位置（相机系）：")
        print("   x = %+.5f     ← 填 TARGET_X   （横向 → 控转向）" % x)
        print("   y = %+.5f     ← 填 TARGET_Y   ★ 关键（上下 → 控前后）" % y)
        print("   z = %+.5f     （距离，仅参考）" % z)
        print("\n★ 前提：此刻【机械臂能夹住物块】✓ 就用这三个数 ✓")
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", type=int, default=0, help="持续观察 N 秒")
    ap.add_argument("--teach", action="store_true")
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--id", type=int, default=3)
    ap.add_argument("--tx", type=float, default=None, help="目标 x（横向）")
    ap.add_argument("--ty", type=float, default=None, help="目标 y（上下→控前后）")
    ap.add_argument("--tz", type=float, default=None, help="目标 z（距离，仅参考）")
    ap.add_argument("--seen", type=float, default=2.0)
    ap.add_argument("--wait", type=float, default=5.0)
    ap.add_argument("--tol_x", type=float, default=0.008)
    ap.add_argument("--tol_y", type=float, default=0.010)
    ap.add_argument("--tol_z", type=float, default=0.020)
    ap.add_argument("--ky", type=float, default=3.0, help="前后增益（y 偏差→速度）")
    ap.add_argument("--step", type=float, default=0.20)
    ap.add_argument("--settle", type=float, default=0.50)
    ap.add_argument("--dir_x", type=float, default=-1.0)
    ap.add_argument("--dir_z", type=float, default=1.0)
    ap.add_argument("--kz", type=float, default=3.0, help="距离增益（米偏差→速度）")
    ap.add_argument("--kx", type=float, default=8.0,
                    help="★ 横向增益（米偏差→角速度）。太小会卡在十几毫米收不动")
    ap.add_argument("--vmax", type=float, default=0.04, help="前进/后退速度上限 m/s")
    ap.add_argument("--vmin", type=float, default=0.02,
                    help="最小前进速度（底盘死区，默认 0.02）")
    ap.add_argument("--min_radius", type=float, default=0.40,
                    help="★ 最小转弯半径 m（曲率约束；底盘实际 0.375，这里留余量用 0.40）")
    ap.add_argument("--coarse_x", type=float, default=0.015,
                    help="★ 粗调阈值 m：横向偏差超过它就先【后退取景】（默认 15mm）")
    ap.add_argument("--coarse_v", type=float, default=0.04,
                    help="★ 粗调后退速度 m/s（默认 0.04）")
    ap.add_argument("--wmax", type=float, default=0.20, help="转向速度上限 rad/s")
    ap.add_argument("--max", type=int, default=20)
    ap.add_argument("--speed_z", type=float, default=0.08)
    ap.add_argument("--speed_x", type=float, default=0.03)
    ap.add_argument("--arm", type=int, default=1)
    a = ap.parse_args()

    rospy.init_node("car_align", anonymous=True, disable_signals=True)
    al = Aligner(a)
    try:
        if a.watch:
            return al.watch(a.watch)
        if a.teach:
            return al.teach()
        return al.run()
    finally:
        al.stop()


if __name__ == "__main__":
    sys.exit(main())
