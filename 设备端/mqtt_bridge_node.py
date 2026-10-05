#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
mqtt_bridge_node.py —— MQTT ↔ ROS 桥接节点（跑在小车上）
==========================================================
作用：把云平台下发的 MQTT 指令，转成小车上的 ROS 动作；动作完成后上报结果。

这是「跨系统协同」的关键实现：云平台不知道 ROS 的存在，小车不知道 MQTT 之外的事，
本节点负责把两边接起来。

【接的 ROS 接口】（全部是现成的话题，不需要改老代码）
  发布 → /move_base_simple/goal  (PoseStamped)      导航目标
  发布 → target_named            (String)           机械臂命名姿态 init/sentry/grap
  发布 → target_pose             (PoseStamped)      机械臂笛卡尔位姿
  订阅 → /move_base/result       (MoveBaseActionResult)   导航结果（status==3 到达）
  订阅 → /move_group/result      (MoveGroupActionResult)  机械臂结果（status==3 完成）

【对应的 MQTT】
  收 → cjy/car-01/cmd     {cmd: "goto_pick"|"deliver", task_id, site, slot, count}
  发 → cjy/car-01/report  {device, role, event, task_id, ...}

【用法】
  DRY_RUN = True   先跑这个（不真导航，只打印 + 假到达）→ 验证 MQTT 链路和状态机
  DRY_RUN = False  地图和坐标标定好之后改用这个 → 真机导航

  python3 mqtt_bridge_node.py --dry      # 干跑
  python3 mqtt_bridge_node.py            # 实跑

依赖：paho-mqtt（已装在 ~/chen_car_drone/libs）
"""

import argparse
import json
import os
import sys
import threading
import time

# ---- 依赖路径 ----
for _p in (os.path.expanduser("~/chen_car_drone/libs"),):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import paho.mqtt.client as mqtt
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from moveit_msgs.msg import MoveGroupActionResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# ★ 夹爪控制用的自定义消息（wd_arm_moveit_demo/ArmJoint）
try:
    from wd_arm_moveit_demo.msg import ArmJoint
    HAS_ARMJOINT = True
except ImportError:
    HAS_ARMJOINT = False

# ==================== 配置 ====================
# ---- 配置读取：优先 config_local.py，其次环境变量（★ 明文密码不进仓库）----
try:
    import config_local as _CFG
except ImportError:
    _CFG = None

def _cfg(key, default=""):
    v = getattr(_CFG, key, None) if _CFG else None
    return v if v not in (None, "") else os.getenv(key, default)

BROKER    = _cfg("MQTT_BROKER", "101.37.242.91")
PORT      = int(_cfg("MQTT_PORT", "1884"))
USERNAME  = _cfg("MQTT_USER_CAR", "cjy-car")
PASSWORD  = _cfg("MQTT_PASS_CAR", "")
DEVICE_ID = _cfg("DEVICE_ID_CAR", "car-01")
NS        = _cfg("MQTT_NS", "cjy")
if not PASSWORD:
    rospy.logerr("[bridge] 未读到密码！请复制 config.example.py 为 config_local.py 并填写 MQTT_PASS_CAR")
    sys.exit(1)

# ★★ 干跑开关：True = 只打印不真导航；False = 真发导航目标
#    2026-10-04 起：地图已建好、坐标已标定 → 改成 False 真跑
DRY_RUN = False

# ★ 站点坐标表（格式：x, y, qz, qw）
#   ⚠️ 换地图/换场地后必须重新标定这里（用 /amcl_pose 或 tf_echo map base_footprint 读）
SITES = {
    "1": {
        # ★ 2026-10-04 家里客厅建图后实测标定（/amcl_pose 读取）
        "pick":    (1.2196, 0.0662, 0.0137, 0.9999),      # 取货点（朝向≈1.6°）
        "deliver": (1.0980, 0.9165, 0.7061, 0.7081),      # 送达点（朝向≈90°）
    },
    # ⚠️ 站点 2/3 尚未标定（以下是从旧图抄的占位值，别用！到学校多站点时再标）
    "2": {"pick": (0.0, 0.0, 0.0, 1.0), "deliver": (0.0, 0.0, 0.0, 1.0)},
    "3": {"pick": (0.0, 0.0, 0.0, 1.0), "deliver": (0.0, 0.0, 0.0, 1.0)},
}

# 机械臂动作序列（照抄 ar_pick_place.cpp 的顺序）
# 特殊项 "__close__" / "__open__" = 控制夹爪
PICK_SEQ = ["init", "sentry", "grap", "__close__", "init"]    # 取货：到位→夹住→抬起
DROP_SEQ = ["grap", "__open__", "init"]                        # 放下：到位→松开→归位


# ==================== 桥接节点 ====================
class CarBridge(object):
    def __init__(self, dry_run):
        self.dry = dry_run
        self.lock = threading.Lock()

        # ---- ROS ----
        rospy.init_node("mqtt_bridge", anonymous=False)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.pub_named = rospy.Publisher("target_named", String, queue_size=5)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.on_nav_result)
        rospy.Subscriber("/move_group/result", MoveGroupActionResult, self.on_arm_result)

        # ---- 夹爪（★ 发 TargetAngle，消息 wd_arm_moveit_demo/ArmJoint）----
        # 夹爪 = joints[5]：张开 30 / 闭合 135（0~180 度制）
        self.arm_angles = [90.0, 90.0, 90.0, 90.0, 90.0, 30.0]     # 初始：张开
        self.pub_gripper = None
        if HAS_ARMJOINT:
            self.pub_gripper = rospy.Publisher("TargetAngle", ArmJoint, queue_size=10)
            # ★ 订阅 MoveIt 的关节状态，保持"当前角度"最新（发夹爪时前 5 个关节不能乱动）
            rospy.Subscriber("/move_group/fake_controller_joint_states",
                             JointState, self.on_joint_state)
        else:
            rospy.logwarn("[bridge] 未找到 wd_arm_moveit_demo/ArmJoint → 夹爪控制不可用")

        # ---- 状态 ----
        self.task = None          # 当前任务 {"task_id","site","slot","count","phase"}
        self.stage = None         # 当前阶段：nav_to_pick / arm_picking / nav_to_deliver

        # ---- MQTT ----
        self.mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=DEVICE_ID)
        self.mq.username_pw_set(USERNAME, PASSWORD)
        self.mq.on_connect = self.on_mqtt_connect
        self.mq.on_message = self.on_mqtt_message
        self.mq.on_disconnect = lambda c, u, f=None, rc=None, p=None: rospy.logwarn(
            "[bridge] MQTT 断开 rc=%s", rc)

        rospy.loginfo("[bridge] 连接 MQTT %s:%s …", BROKER, PORT)
        self.mq.connect(BROKER, PORT, keepalive=60)
        self.mq.loop_start()                       # MQTT 跑在自己线程里
        rospy.loginfo("[bridge] 就绪（DRY_RUN=%s）", self.dry)
        if self.dry:
            rospy.logwarn("[bridge] ★ 干跑模式：不会真发导航目标，用定时器模拟到达")

    # ---------- MQTT ----------
    def on_mqtt_connect(self, c, u, f, rc, p=None):
        if rc == 0:
            c.subscribe("%s/%s/cmd" % (NS, DEVICE_ID))
            rospy.loginfo("[bridge] MQTT 已连接，订阅 %s/%s/cmd", NS, DEVICE_ID)
            self.report("idle", note="bridge started")     # ★ 声明空闲 → 平台可派单
        else:
            rospy.logerr("[bridge] MQTT 连接失败 rc=%s", rc)

    def on_mqtt_message(self, c, u, msg):
        try:
            d = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return
        cmd = d.get("cmd")
        rospy.loginfo("[bridge] 收到指令：%s", d)
        if cmd == "goto_pick":
            self.start_pick(d)
        elif cmd == "deliver":
            self.start_deliver(d)
        else:
            rospy.logwarn("[bridge] 未知指令 %s", cmd)

    def report(self, event, **kw):
        msg = {"device": DEVICE_ID, "role": "car", "event": event, "ts": int(time.time())}
        msg.update(kw)
        topic = "%s/%s/report" % (NS, DEVICE_ID)
        self.mq.publish(topic, json.dumps(msg, ensure_ascii=False))
        rospy.loginfo("[bridge] → 上报到频道 %s：%s", topic, msg)

    # ---------- 任务流程 ----------
    def start_pick(self, d):
        site = str(d.get("site"))
        cfg = SITES.get(site)
        if not cfg:
            rospy.logerr("[bridge] 未知站点 %s", site)
            self.report("exception", task_id=d.get("task_id"), reason="unknown site")
            return
        with self.lock:
            self.task = {"task_id": d.get("task_id"), "site": site,
                         "slot": d.get("slot"), "count": d.get("count")}
            self.stage = "nav_to_pick"
        self.publish_goal(cfg["pick"], "取货点站点%s" % site)

    def start_deliver(self, d):
        with self.lock:
            if self.task:
                self.task["task_id"] = d.get("task_id", self.task.get("task_id"))
                site = self.task["site"]
            else:
                site = "1"
                self.task = {"task_id": d.get("task_id"), "site": site}
            self.stage = "nav_to_deliver"
        self.publish_goal(SITES[site]["deliver"], "送达点站点%s" % site)

    def publish_goal(self, pose7, label):
        x, y, qz, qw = pose7
        if self.dry:
            rospy.logwarn("[bridge·干跑] 本应导航到 %s：x=%.3f y=%.3f qz=%.3f qw=%.3f",
                          label, x, y, qz, qw)
            threading.Timer(3.0, self._fake_arrive).start()      # 3 秒后假装到达
            return
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        self.pub_goal.publish(p)
        rospy.loginfo("[bridge] 已下发导航目标：%s", label)

    def _fake_arrive(self):
        """干跑模式：模拟 move_base 到达"""
        rospy.logwarn("[bridge·干跑] 假装导航到达")
        self.on_nav_result_ok()

    # ---------- 导航结果 ----------
    def on_nav_result(self, msg):
        if msg.status.status == 3:
            self.on_nav_result_ok()
        else:
            rospy.logwarn("[bridge] 导航失败 status=%d", msg.status.status)
            self.report("exception", task_id=(self.task or {}).get("task_id"),
                        reason="navigation failed %d" % msg.status.status)

    def on_nav_result_ok(self):
        with self.lock:
            stage = self.stage
        if stage == "nav_to_pick":
            rospy.loginfo("[bridge] 已到达取货点")
            self.report("arrived", task_id=self.task.get("task_id"),
                        site=self.task.get("site"))
            self.stage = "arm_picking"
            self.arm_seq(PICK_SEQ[:])          # 依次做 init → sentry → grap
        elif stage == "nav_to_deliver":
            rospy.loginfo("[bridge] 已到达送达点")
            self.stage = "arm_dropping"
            self.arm_seq(DROP_SEQ[:])          # grap → 松开夹爪 → 归位
        else:
            rospy.logwarn("[bridge] 到达但阶段未知（%s）", stage)

    # ---------- 夹爪 ----------
    def on_joint_state(self, msg):
        """同步 MoveIt 的关节状态（换算方式照抄 SimulationToMachine.cpp）"""
        if len(msg.position) == 5:
            for i in range(5):
                self.arm_angles[i] = (180 / 3.14) * msg.position[i] + 90
        elif len(msg.position) == 1:
            self.arm_angles[5] = ((180 - 30) / 90.0) * (msg.position[0] * (180 / 3.14) + 90) + 30

    def set_gripper(self, close):
        """控制夹爪：close=True 闭合(joints[5]=135) / False 张开(=30)"""
        if self.dry:
            rospy.logwarn("[bridge·干跑] 本应%s夹爪", "闭合" if close else "张开")
            threading.Timer(2.0, self._arm_done).start()
            return
        if self.pub_gripper is None:
            rospy.logerr("[bridge] 夹爪不可用（缺 ArmJoint 消息）")
            self._arm_done()
            return
        a = list(self.arm_angles)          # ★ 前 5 个关节保持当前值，只改夹爪
        a[5] = 135.0 if close else 30.0
        m = ArmJoint()
        m.joints = a
        m.run_time = 1000
        self.pub_gripper.publish(m)
        rospy.loginfo("[bridge] 夹爪 → %s (joints[5]=%.0f)",
                      "闭合" if close else "张开", a[5])
        # TargetAngle 没有完成回执 → 等 2 秒当作完成
        threading.Timer(2.0, self._arm_done).start()

    # ---------- 机械臂 ----------
    def arm_named(self, name):
        if self.dry:
            rospy.logwarn("[bridge·干跑] 本应让机械臂做姿态：%s", name)
            threading.Timer(2.0, self._arm_done).start()
            return
        s = String()
        s.data = name
        self.pub_named.publish(s)
        rospy.loginfo("[bridge] 机械臂 → %s", name)

    def arm_seq(self, seq):
        """依次执行姿态序列，最后收尾"""
        self._seq = list(seq)
        self._arm_seq_next()

    def _arm_seq_next(self):
        """执行动作序列的下一步；序列走完按当前阶段收尾"""
        if not self._seq:
            if self.stage == "arm_picking":
                rospy.loginfo("[bridge] 取货动作完成")
                self.report("pick_done", task_id=self.task.get("task_id"),
                            slot=self.task.get("slot"), count=self.task.get("count"))
                self.stage = "wait_deliver"
            elif self.stage == "arm_dropping":
                rospy.loginfo("[bridge] 已放下，任务完成")
                self.report("delivered", task_id=self.task.get("task_id"))
                self.task = None
                self.stage = None
                self.report("idle")            # ★ 空闲了，可以接下一个任务
            return
        step = self._seq.pop(0)
        if step == "__close__":
            self.set_gripper(True)
        elif step == "__open__":
            self.set_gripper(False)
        else:
            self.arm_named(step)

    def on_arm_result(self, msg):
        if msg.status.status == 3:
            if self.stage in ("arm_picking", "arm_dropping"):
                self._arm_seq_next()
        elif msg.status.status == 4:
            rospy.logwarn("[bridge] 机械臂规划失败")

    def _arm_done(self):
        """干跑模式：模拟机械臂动作完成"""
        self.on_arm_result(type("M", (), {"status": type("S", (), {"status": 3})()})())

    def spin(self):
        rospy.spin()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true", help="干跑模式（不真导航）")
    args = ap.parse_args()
    if args.dry:
        global DRY_RUN
        DRY_RUN = True
    try:
        CarBridge(DRY_RUN).spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

# ============================================================
# 已查证的接口（2026-10-03）
# · 夹爪：发 wd_arm_moveit_demo/ArmJoint 到话题 "TargetAngle"
#          joints[5] = 135 闭合 / 30 张开（0~180 度制）
#          定义见 SimulationToMachine.cpp 的 pub_close() / pub_open()
#          注意：前 5 个关节要填"当前值"，否则机械臂会跳 → 本节点已订阅
#                /move_group/fake_controller_joint_states 保持同步
#
# 待办
# 1) ★ 站点坐标：重建地图后必须重新标定 SITES 里的 pick / deliver
#    （当前值抄自 ar_pick_place.cpp 的旧坐标，且那张地图已被建坏）
# 2) 取货时是否要 AR 码识别：原 ar_pick_place.cpp 会等 AR 码，桥接版暂时直接抓
# 3) 是否要做成开机自启（systemd / rc.local）
# ============================================================
