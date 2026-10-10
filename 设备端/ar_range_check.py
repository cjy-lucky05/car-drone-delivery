#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AR 码识别范围测试节点 —— 用来确定物块该摆在哪个位置

【干什么】
  实时打印相机看到了几个 AR 码、码号、距离、以及码在【相机坐标系】和
  【车体坐标系】里的位置；跑一段时间后按 Ctrl+C，会打印一份范围总结
  （最近/最远识别距离、车体系里的 x/y/z 范围）→ 照着它摆物块。

【怎么跑】（小车上，先起 roscore + base_control + wd_vision bringup）
    source ~/catkin_ws/devel/setup.bash
    python3 ar_range_check.py
    # 想同时看相机画面（推荐，比 RViz 靠谱）另开一个终端：
    #   rosrun image_view image_view image:=/arm_camera/image_raw
    # 或：rqt_image_view

【怎么用】
  1. 让机械臂停在 sentry 姿态（相机朝下那个姿态）：
       rostopic pub -1 /TargetAngle wd_arm_moveit_demo/ArmJoint "{joints: [90, 90, 98, -40, 90, 30], run_time: 2200}"
  2. 把物块摆到取货位置，前后左右慢慢挪，看终端打印
  3. 记下"能稳定识别到"的位置范围 → 把物块摆在这个范围里
  4. Ctrl+C 看总结

【判据】
  · 车体系 x/y 就是相对车中心的水平位置（x 前、y 左）
  · 机械臂 grap 姿态是按"车头前方约 20cm、离地约 10cm"定的
    → 物块摆在 x≈0.15~0.25、y≈-0.05~+0.05 附近最稳
  · 看不到码时先查：物块太近/太远/太歪/反光/太小（码是 2.5cm）
"""
import sys
import time
import math
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PointStamped
import tf


def _dist(p):
    return math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)


class ARRangeCheck(object):
    def __init__(self, hz=2.0):
        self.hz = hz
        self.last_print = 0.0
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        # ★★ ar_track_alvar 的消息 header.frame_id 可能是【空的】→ 用 launch 里
        #    配的 output_frame 兜底（wd_vision/launch/ar_track.launch: output_frame=arm_image_link）
        self.src_frame = rospy.get_param("~source_frame", "arm_image_link")
        self.listener = tf.TransformListener()
        self._tf_err = None            # 只打印一次的 TF 报错

        self.last_msg = 0.0                  # 最后一次收到 /ar_pose_marker 消息的时间
        self.n_seen = 0                      # 看到码的消息帧数
        self.ids_seen = {}                   # 码号 → 出现次数
        self.dr = [None, None]               # 距离范围
        self.xr = [None, None]               # 车体系 x 范围
        self.yr = [None, None]               # 车体系 y 范围
        self.zr = [None, None]               # 车体系 z 范围
        self.t_start = time.time()

        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.on_marker, queue_size=10)
        # ★★ ar_track_alvar 只在【看到码】时才发消息 —— 看不到码时一条都没有，
        #    所以必须自己起个定时器提示，否则用户会以为程序挂了 ✗
        rospy.Timer(rospy.Duration(2.0), self._tick)
        print("=" * 78)
        print(" AR 码识别范围测试   等 /ar_pose_marker …（物块放到取货位，慢慢挪动）")
        print(" 停止：Ctrl+C（会打印一份范围总结）")
        print("=" * 78)

    # ---------- 工具 ----------
    @staticmethod
    def _rng(r, v):
        """把一个值并进 [min, max] 区间"""
        if r[0] is None or v < r[0]:
            r[0] = v
        if r[1] is None or v > r[1]:
            r[1] = v

    def _to_base(self, pose):
        """相机系 → 车体系，返回 (x,y,z)；TF 没就绪时返回 None（不抛）

        ★ 两个兜底（实测踩过）：
          ① header.frame_id 为空 → 用 self.src_frame（= launch 的 output_frame）
             （症状：刷 "tf2 frame_ids cannot be empty"）
          ② 该时间戳的 TF 还没有 → 改用"最新可用"（stamp=0）
        """
        for latest in (False, True):
            try:
                pt = PointStamped()
                pt.header = pose.header
                if not pt.header.frame_id:
                    pt.header.frame_id = self.src_frame
                if latest:
                    pt.header.stamp = rospy.Time(0)
                pt.point = pose.pose.position
                q = self.listener.transformPoint(self.base_frame, pt)
                return (q.point.x, q.point.y, q.point.z)
            except Exception as e:
                if self._tf_err is None:                 # 只提示一次，别刷屏
                    self._tf_err = str(e).strip().split("\n")[0]
                    print("[!] 车体系换算失败：%s" % self._tf_err)
                    print("    → 检查坐标变换：rosrun tf tf_echo %s %s"
                          % (self.base_frame, self.src_frame))
        return None

    # ---------- 定时提示 ----------
    def _tick(self, _evt=None):
        """3 秒没收到任何 /ar_pose_marker 消息 → 提示（限速，别刷屏）"""
        if time.time() - max(self.last_msg, self.t_start) < 3.0:
            return
        print("[--:--:--] 当前没看到 AR 码 ✗" + ("（本次已识别过 %d 帧）" % self.n_seen if self.n_seen else ""))
        if not self.n_seen:
            print("            检查：① 物块摆到取货位了吗 ② 相机在发图吗"
                  "（rostopic hz /arm_camera/image_raw）")
            print("                  ③ wd_vision bringup 起了吗"
                  "（rostopic list | grep ar_pose_marker）")
        self.last_msg = time.time()

    # ---------- 回调 ----------
    def on_marker(self, msg):
        now = time.time()
        self.last_msg = now
        if not msg.markers:
            return

        # ★★★ 范围统计覆盖【视野里所有码】—— 这才代表相机的真实可识别范围
        #     （打印的"当前位置"另取最近的那个，见下）
        best, best_d, best_xyz = None, 1e9, None
        for m in msg.markers:
            p = m.pose.pose.position
            d = _dist(p)
            self.ids_seen[int(m.id)] = self.ids_seen.get(int(m.id), 0) + 1
            self._rng(self.dr, d)
            xyz = self._to_base(m.pose)
            if xyz:
                self._rng(self.xr, xyz[0])
                self._rng(self.yr, xyz[1])
                self._rng(self.zr, xyz[2])
            if d < best_d:                       # 最近的 = 当前要摆的那一件
                best, best_d, best_xyz = m, d, xyz
        self.n_seen += 1

        if now - self.last_print < 1.0 / max(self.hz, 0.1):
            return
        self.last_print = now
        self._print(len(msg.markers), best, best_d, best_xyz)

    def _print(self, n, best, best_d, xyz):
        p = best.pose.pose.position
        print("─" * 78)
        print("[%s] 看到 %d 个码 | 目标 ★%d 号   距离 %.3f m"
              % (time.strftime("%H:%M:%S"), n, int(best.id), best_d))
        print("          相机系  x=%+.3f  y=%+.3f  z=%+.3f   (相机看到的位置)"
              % (p.x, p.y, p.z))
        if xyz:
            print("          车体系  x=%+.3f  y=%+.3f  z=%+.3f   (相对车中心: x前 y左 z上)"
                  % xyz)
            print("          摆放参考 x≈0.15~0.25, y≈-0.05~+0.05  ← 机械臂 grap 对准的位置")
        else:
            print("          车体系  换算失败（TF 还没就绪？）")
        print("          [范围] 距离 %.2f~%.2f m" % tuple(self.dr))

    # ---------- 收尾 ----------
    def summary(self):
        print("\n" + "=" * 78)
        print(" 识别范围总结（本次跑了 %.0f 秒）" % (time.time() - self.t_start))
        print("=" * 78)
        if self.n_seen == 0:
            print(" ✗ 整段时间一次都没识别到码")
            print("   排查顺序：① 物块上是不是贴了码（2.5cm）② 相机画面能看到物块吗")
            print("             ③ 距离/角度/反光 ④ 起 wd_vision bringup 了吗")
            return
        print(" 识别到码的帧数：%d" % self.n_seen)
        print(" 识别到的码号  ：%s" % (sorted(self.ids_seen) or "—"))
        print(" 距离范围      ：%.3f ~ %.3f m" % tuple(self.dr))
        if self.xr[0] is not None:
            print(" 车体系 x 范围 ：%+.3f ~ %+.3f m" % tuple(self.xr))
            print(" 车体系 y 范围 ：%+.3f ~ %+.3f m" % tuple(self.yr))
            print(" 车体系 z 范围 ：%+.3f ~ %+.3f m" % tuple(self.zr))
        print("-" * 78)
        print(" ★ 结论：把物块摆在上面 x/y 范围【居中】的位置，且距离在中间值附近，")
        print("   既保证相机认得出、又保证机械臂 grap 够得着 ✓")


def main():
    rospy.init_node("ar_range_check", anonymous=True)
    node = ARRangeCheck(hz=float(sys.argv[1]) if len(sys.argv) > 1 else 2.0)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    node.summary()


if __name__ == "__main__":
    main()
