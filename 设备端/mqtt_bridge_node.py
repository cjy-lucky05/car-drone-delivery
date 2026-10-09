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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
from move_base_msgs.msg import MoveBaseActionResult
from moveit_msgs.msg import MoveGroupActionResult
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# ★ 机械臂控制用的自定义消息（wd_arm_moveit_demo/ArmJoint）
try:
    from wd_arm_moveit_demo.msg import ArmJoint
    HAS_ARMJOINT = True
except ImportError:
    ArmJoint = None
    HAS_ARMJOINT = False

# ★★ AR 码识别（ar_track_alvar 输出）—— 用于"识别到货物才抓"
try:
    from ar_track_alvar_msgs.msg import AlvarMarkers
except ImportError:
    AlvarMarkers = None

# ==================== 配置 ====================
# ---- 配置读取：优先 config_local.py，其次环境变量（★ 明文密码不进仓库）----
try:
    import config_local as _CFG
except ImportError:
    _CFG = None

def _cfg(key, default=""):
    v = getattr(_CFG, key, None) if _CFG else None
    return v if v not in (None, "") else os.getenv(key, default)

BROKER    = _cfg("MQTT_BROKER", "你的MQTT服务器IP")
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
#   ★ 2026-10-07 已按实验室新图重新标定
SITES = {
    "1": {
        # ★★★ 2026-10-09 【新实验室】建图 lab_20261009 后实测标定（/amcl_pose 读取）
        #     旧实验室的坐标已作废（地图原点 = 建图起点，换场地必重标）
        "pick":    (1.3498, 0.2309, -0.045680, 0.998956),   # 取货点（yaw≈-5.2°）
        "deliver": (2.0260, -1.7208, -0.994891, 0.100955),  # 送达点（yaw≈-168.4°）
    },
    # ⚠️ 站点 2/3 尚未标定（占位值，别用！多站点时再标）
    "2": {"pick": (0.0, 0.0, 0.0, 1.0), "deliver": (0.0, 0.0, 0.0, 1.0)},
    "3": {"pick": (0.0, 0.0, 0.0, 1.0), "deliver": (0.0, 0.0, 0.0, 1.0)},
}

# ==================== ★★★ 机械臂舵机姿态表（2026-10-08）====================
#   单位：舵机原始角度（0~180/250 制，不是弧度）
#   依据：/joint_states 读出 arm_joint1 = 0.0 → 机械臂【已装正】
#         → 所以直接用【原厂定义】的姿态值（不需要 +90 修正）
#   映射：舵机值 = ROS弧度 × 180/π + 90
#   原厂 SRDF：init(0,-1.8,2.446,-2.3726,0)  sentry(0,0,0.1385,-2.2726,0)
#              grap(0,-0.755,-1.5708,0.0114,-0.03)  close(grip=0) open(grip=-1.54)
#   ⚠️ 超范围的值会被底盘固件【静默丢弃】（整条指令都不执行）→ 别乱填
ARM_POSES = {
    # ⚠️ 注意最后一位 = 夹爪（30=张开 / 135=闭合）；抓完货回 init 要保留夹爪值
    "init":   [90.0, -13.0, 230.0, -46.0, 90.0, 30.0],   # 折叠（开机基准）
    "sentry": [90.0,  90.0,  98.0, -40.0, 90.0, 30.0],   # 抬起待命
    "grap":   [90.0,  47.0,   0.0,  91.0, 88.0, 30.0],   # 伸下去抓
    "up":     [90.0,  90.0,  90.0,  90.0, 90.0, 30.0],   # 全中位（立直）
}

# ==================== ★ 动作速度（毫秒；值越大越慢）====================
#   run_time = 舵机从当前位置走到目标位置用的时间
#   ★ 调慢一点更稳：夹货时太快容易把货打飞/没夹正；放货时太快容易甩出去
#   想更快/更慢改这里的默认值，或者跑的时候加环境变量：
#     ARM_RUN_TIME=3000 GRIP_RUN_TIME=4000 python3 -u mqtt_bridge_node.py
ARM_RUN_TIME  = int(os.getenv("ARM_RUN_TIME",  "2200"))   # 大臂/整体姿态（原 1200）
GRIP_RUN_TIME = int(os.getenv("GRIP_RUN_TIME", "3000"))   # 夹爪开合（原 1000）★ 最慢
ARM_WAIT      = float(os.getenv("ARM_WAIT",   "2.5"))     # 每个姿态后等多久再发下一条

# ==================== ★ 起点（送完货回位）====================
#   小车送完货后自动导航回这个点。
#   ⚠️ 换成你的实际起点坐标（就是车平时停放/开机的位置，看 RViz 里车的初始位置）
#   想不回位：把 GO_HOME 设成 False
#   ★ 手动：网页「手动操作 → 返回起始点」随时可用
#   ★★ 自动：送完货后自动回位（不想要就把 AUTO_GO_HOME 设 0）
# ★ 2026-10-09 用户定：送完货【不自动】回起点 —— 车就停在送达点待命
#   要回起点：网页【🎮 小车手动操作】→ 选「返回起始点」→ 发送指令 ✓
#   想恢复自动：AUTO_GO_HOME=1 python3 -u mqtt_bridge_node.py
AUTO_GO_HOME = os.getenv("AUTO_GO_HOME", "0").strip() not in ("0", "false", "False", "no")
# ★★★ 等 AR 码的最长时间（秒）：车可能比人/无人机先到，货还没摆上去
#     2026-10-09 由 15 秒改成 120 秒（用户要求）：给人留出"把物块摆到取货点"的时间 ✓
#     想改回去：AR_WAIT_SEC=15 python3 -u mqtt_bridge_node.py
AR_WAIT_SEC = float(os.getenv("AR_WAIT_SEC", "120"))
HOME_POS = (float(os.getenv("HOME_X", "-0.1222")),     # x  ★ 2026-10-09 新实验室实测
            float(os.getenv("HOME_Y", "0.3362")),      # y
            float(os.getenv("HOME_Z", "0.0")),         # z
            float(os.getenv("HOME_YAW", "0.3072")))    # yaw(弧度) ≈ 17.6°

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
        # ★ 实时位置（订阅 AMCL 定位结果，每秒上报给平台 → 网页地图用）
        self.pos = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.on_amcl_pose)
        rospy.Timer(rospy.Duration(1.0), self.report_position)      # 每 1 秒上报一次
        self.pub_named = rospy.Publisher("target_named", String, queue_size=5)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.on_nav_result)
        rospy.Subscriber("/move_group/result", MoveGroupActionResult, self.on_arm_result)

        # ★★ AR 码识别：ar_track_alvar 发 /ar_pose_marker → 记录"最近一次看到货"
        self.holding = False           # ★ 夹爪是否正夹着货（防止被状态覆盖）
        self.home_override = None      # ★ 手动回位时可指定坐标
        self._marker_id = None         # ★★ 最后一次读到的 AR 码【序号】(= 货物身份证)
        self._expect_marker = None     # ★★★ 本次任务【期望】的码号（平台派单时下发 → 抓前校验）
        self._marker_ts = 0.0          # 最后一次看到 AR 码的时间
        self._marker_dist = None       # 距相机的距离(m)
        # ★ AR_REQUIRED=1（默认）：必须先识别到 AR 码才抓；=0：识别不可用时直接抓
        self.ar_required = os.getenv("AR_REQUIRED", "1").strip() not in ("0", "false", "False", "no")
        self._has_ar = False
        if AlvarMarkers is not None and not self.dry:
            try:
                rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.on_marker)
                self._has_ar = True
                rospy.loginfo("[bridge] ★ AR 识别已订阅（/ar_pose_marker）"
                              "—— AR_REQUIRED=%s", self.ar_required)
            except Exception as e:
                rospy.logwarn("[bridge] 订阅 AR 失败：%s", e)
        else:
            rospy.logwarn("[bridge] AR 识别不可用（缺 ar_track_alvar_msgs 或干跑模式）")

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
        # ★★ 电量上报（2026-10-08 新增）
        self._bat_pct = None      # 百分比
        self._bat_v = None        # 电压
        self._bat_type = None
        self._setup_battery()
        try:
            rospy.Timer(rospy.Duration(30.0), self._tick_battery)   # 每 30 秒报一次
        except Exception:
            pass

    # ---------- ★★★ 电量 ----------
    def _setup_battery(self):
        """自适应订阅电量话题（不同底盘的 /battery 类型不一样，先探测再订阅）"""
        try:
            import subprocess
            t = subprocess.check_output(["rostopic", "type", "/battery"],
                                        stderr=subprocess.STDOUT, timeout=8).decode().strip()
        except Exception as e:
            rospy.logwarn("[bridge] 探测 /battery 失败（将不上报电量）：%s", e)
            return
        rospy.loginfo("[bridge] /battery 类型 = %s", t)
        try:
            if "BatteryState" in t:
                from sensor_msgs.msg import BatteryState
                self._bat_type = "BatteryState"
                rospy.Subscriber("/battery", BatteryState, self.on_battery)
            elif "Float32" in t:
                from std_msgs.msg import Float32
                self._bat_type = "Float32"
                rospy.Subscriber("/battery", Float32, self.on_battery)
            elif "Int32" in t:
                from std_msgs.msg import Int32
                self._bat_type = "Int32"
                rospy.Subscriber("/battery", Int32, self.on_battery)
            else:
                rospy.logwarn("[bridge] 未知电量类型 %s（可手工改 _setup_battery）", t)
        except Exception as e:
            rospy.logwarn("[bridge] 订阅电量失败：%s", e)

    def on_battery(self, msg):
        """记录电量（不直接上报，交给 30 秒定时器节流上报）"""
        try:
            if self._bat_type == "BatteryState":
                v = getattr(msg, "voltage", None)
                pct = getattr(msg, "percentage", None)
                if pct is not None and pct <= 1.0:
                    pct = pct * 100.0
                if v: self._bat_v = float(v)
                if pct is not None: self._bat_pct = float(pct)
            else:
                x = float(getattr(msg, "data", 0) or 0)
                # ★ 约定：>100 当电压(V)，<=100 当百分比
                if x > 100: self._bat_v = x
                else:       self._bat_pct = x
        except Exception:
            pass

    def _tick_battery(self, evt=None):
        if self._bat_pct is None and self._bat_v is None:
            return
        self.report("battery", quiet=True,
                    percent=(round(self._bat_pct, 1) if self._bat_pct is not None else None),
                    voltage=(round(self._bat_v, 2) if self._bat_v is not None else None),
                    name="小车1号")

    def on_mqtt_connect(self, c, u, f, rc, p=None):
        if rc == 0:
            c.subscribe("%s/%s/cmd" % (NS, DEVICE_ID))
            rospy.loginfo("[bridge] MQTT 已连接，订阅 %s/%s/cmd", NS, DEVICE_ID)
            self.report("idle", note="bridge started")     # ★ 声明空闲 → 平台可派单
        else:
            rospy.logerr("[bridge] MQTT 连接失败 rc=%s", rc)

    # ---------- ★ 实时位置 ----------
    def on_amcl_pose(self, msg):
        """收到 AMCL 定位 → 记住车当前位姿（四元数转成 yaw）"""
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pos = (p.position.x, p.position.y, yaw)

    def report_position(self, evt=None):
        """每秒把车的位置上报给平台（网页在地图上画车用）"""
        with self.lock:
            pos = self.pos
        if not pos:
            return
        x, y, yaw = pos
        self.report("position", quiet=True, x=round(x, 3), y=round(y, 3), yaw=round(yaw, 3))

    def on_mqtt_message(self, c, u, msg):
        try:
            d = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return
        cmd = d.get("cmd")
        rospy.loginfo("[bridge] 收到指令：%s", d)
        if cmd == "goto_pick":
            self.start_pick(d)
        elif cmd == "pick":
            # ★ 平台确认"货已入库"后才发这个 → 此时才动机械臂
            self.start_arm_pick(d)
        elif cmd == "wait":
            # ★ 货还没入库 → 原地等着（不动机械臂）
            rospy.logwarn("[bridge] 平台要求原地等待：%s（货物尚未入库）", d.get("reason", ""))
        elif cmd == "deliver":
            self.start_deliver(d)
        elif cmd == "go_home":
            # ★★★ 返回起始点（网页手动点，或平台自动派）
            #     指令里带 x/y 就用指令的，否则用 HOME_POS
            if d.get("x") is not None and d.get("y") is not None:
                try:
                    self.home_override = (float(d["x"]), float(d["y"]),
                                          float(d.get("yaw") or 0.0))
                    rospy.loginfo("[bridge] 回位目标改为指令指定 (%.2f, %.2f)",
                                  self.home_override[0], self.home_override[1])
                except Exception:
                    self.home_override = None
            else:
                self.home_override = None
            self.go_home()
        else:
            rospy.logwarn("[bridge] 未知指令 %s", cmd)

    def report(self, event, quiet=False, **kw):
        """上报事件。★ quiet=True 时不打日志（位置这类高频上报用）"""
        msg = {"device": DEVICE_ID, "role": "car", "event": event, "ts": int(time.time())}
        msg.update(kw)
        topic = "%s/%s/report" % (NS, DEVICE_ID)
        self.mq.publish(topic, json.dumps(msg, ensure_ascii=False))
        if not quiet:
            rospy.loginfo("[bridge] → 上报到频道 %s：%s", topic, msg)
        else:
            rospy.loginfo_throttle(10, "[bridge] → 位置上报中：x=%.2f y=%.2f yaw=%.2f",
                                   kw.get("x", 0), kw.get("y", 0), kw.get("yaw", 0))

    # ---------- 任务流程 ----------
    def start_pick(self, d):
        site = str(d.get("site"))
        cfg = SITES.get(site)
        if not cfg:
            rospy.logerr("[bridge] 未知站点 %s", site)
            self.report("exception", task_id=d.get("task_id"), reason="unknown site")
            return
        # ★★★ 记下【这次要取哪一号货】（平台派单时锁定的 AR 码序号）
        #     抓取前会用它做校验 → 防"中途被调包"/"取错件"
        _mk = d.get("marker_id")
        try:
            _mk = int(_mk) if _mk not in (None, "", "None") else None
        except Exception:
            _mk = None
        self._expect_marker = _mk
        if _mk is not None:
            rospy.loginfo("[bridge] ★ 本任务要取 ★%s 号货（抓前会核对）", _mk)
        with self.lock:
            self.task = {"task_id": d.get("task_id"), "site": site,
                         "slot": d.get("slot"), "count": d.get("count"),
                         "marker_id": _mk}
            self.stage = "nav_to_pick"
        self.publish_goal(cfg["pick"], "取货点站点%s" % site)

    def start_arm_pick(self, d=None):
        """★ 平台确认货物已入库 → 才开始机械臂取货

        为什么单独一步（而不是到达就抓）：
          车可能比无人机先到位（车快、无人机要飞过来），此时库位还是空的。
          到点就抓 = 抓空气。正确做法是：车到位先等，货入库后平台再发 pick。
        """
        with self.lock:
            if self.stage != "wait_pick_cmd":
                rospy.logwarn("[bridge] 当前阶段是 %s，忽略 pick 指令", self.stage)
                return
            self.stage = "arm_picking"
        # ★ 以【指令里带的 slot/task_id】为准（平台分配的才是真的）；
        #   没有才退回任务里缓存的（修复：曾出现指令说 1B、上报却是 1A 的问题）
        _d = d or {}
        if _d.get("marker_id") not in (None, "", "None"):
            try:
                self._expect_marker = int(_d["marker_id"])
            except Exception:
                pass
        if self.task:
            if _d.get("slot"):
                self.task["slot"] = _d["slot"]
            if _d.get("task_id"):
                self.task["task_id"] = _d["task_id"]
        slot_now = _d.get("slot") or (self.task or {}).get("slot")
        rospy.loginfo("[bridge] 平台确认货物已入库 %s → 机械臂开始取货", slot_now)
        # ★★ 走"先抬起(相机朝下) → 等 AR 码 → 识别到才抓"的流程
        threading.Thread(target=self._pick_with_ar, daemon=True).start()

    def _pick_with_ar(self):
        """★★★ 识别到 AR 码才抓（防止抓空气）

        流程：init(折叠) → sentry(抬起，相机朝下) → ★等 AR 码(最多 15 秒)
              → 识别到 → grap(伸下去) → 夹爪闭合 → init(收回) → 完成
              → 没识别到 → 不抓，上报 exception
        """
        tid = (self.task or {}).get("task_id")
        slot = (self.task or {}).get("slot")

        self._pub_pose("init")
        time.sleep(ARM_WAIT)
        self._pub_pose("sentry")          # 抬起，相机朝下对着取货位
        time.sleep(ARM_WAIT)

        if self._has_ar and self.ar_required:
            # ★★★ 货物校验（2026-10-09）：
            #   任务带了"期望码号"→ 只认那一个号；读到别的号【不抓】，
            #   等于"抓之前先核对身份"，能发现中途被调包/放错格 ✓
            expect = self._expect_marker
            if expect is not None:
                rospy.loginfo("[bridge] ★ 等待 AR 码（最多 %.0f 秒）… 本次只认 ★%s 号（抓前核对）",
                              AR_WAIT_SEC, expect)
            else:
                rospy.loginfo("[bridge] ★ 等待 AR 码识别（最多 %.0f 秒）…（任务未指定序号，不校验）",
                              AR_WAIT_SEC)
            t0 = time.time()
            ok = False
            wrong_id = None
            while time.time() - t0 < AR_WAIT_SEC:
                if time.time() - self._marker_ts < 1.0:      # 1 秒内看到过码
                    mid = self._marker_id
                    if expect is None:
                        ok = True
                        break
                    if mid is not None and int(mid) == int(expect):
                        ok = True
                        break
                    wrong_id = mid          # ★ 是别的号 → 记下来，继续等"对的那件"
                rospy.sleep(0.3)
            if not ok:
                self._pub_pose("init")
                if expect is not None and wrong_id is not None:
                    why = ("货物校验失败：期望 ★%s 号，实际读到 ★%s 号"
                           "（疑似中途被调包 / 放错货格，已拒绝抓取）" % (expect, wrong_id))
                    rospy.logerr("[bridge] 🚨🚨 %s", why)
                else:
                    why = ("等了 %.0f 秒没在货架上识别到货（AR码）—— 可能【无人机放错货架了】"
                           "或该格本来就是空的，已放弃抓取" % AR_WAIT_SEC)
                    rospy.logwarn("[bridge] ⚠️ %.0f 秒内没识别到 AR 码 → 放弃抓取"
                                  "（可能无人机放错货架 / 该格为空）", AR_WAIT_SEC)
                self.report("exception", task_id=tid, slot=slot, reason=why)
                with self.lock:
                    self.stage = "wait_deliver"        # 交给平台决定下一步
                return
            rospy.loginfo("[bridge] ✅ 识别到 AR 码 ★%s 号（距离 %.2f m）→ 货物校验通过 → 开始抓取",
                          self._marker_id if self._marker_id is not None else "?",
                          self._marker_dist or 0.0)

        self._pub_pose("grap")
        time.sleep(ARM_WAIT)
        self._pub_grip(True)              # ★ 闭合夹爪（夹住货）
        time.sleep(ARM_WAIT)
        self._pub_pose("init", keep_grip=True)   # ★★ 回折叠姿态，但【夹爪保持闭合】
        time.sleep(ARM_WAIT)
        rospy.loginfo("[bridge] 取货动作完成（夹爪保持夹住）")
        self.report("car_load", task_id=tid, slot=slot,
                    count=(self.task or {}).get("count"),
                    marker_id=self._marker_id,          # ★★ 摄像头读到的货物序号
                    tracking_no=(self.task or {}).get("tracking_no") or "")
        self.report("pick_done", task_id=tid, slot=slot,
                    count=(self.task or {}).get("count"))
        with self.lock:
            self.stage = "wait_deliver"

    def start_deliver(self, d):
        with self.lock:
            if self.task:
                self.task["task_id"] = d.get("task_id", self.task.get("task_id"))
                site = self.task["site"]
            else:
                site = "1"
                self.task = {"task_id": d.get("task_id"), "site": site}
            self.stage = "nav_to_deliver"
        # ★★ 上报"出发配送"
        self.report("car_depart", task_id=(self.task or {}).get("task_id"))
        self.publish_goal(SITES[site]["deliver"], "送达点站点%s" % site)

    def publish_goal(self, pose7, label):
        # ★ 兼容两种写法：(x,y,qz,qw) 和 (x,y,z,qz,qw)
        #   （go_home 传的是 5 个，以前会崩：too many values to unpack ✗）
        if len(pose7) >= 5:
            x, y, z, qz, qw = pose7
        else:
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

    def go_home(self):
        """★ 送完货回起点（起点坐标见上面的 HOME_POS）"""
        import math as _m
        x, y, z, yaw = HOME_POS
        ov = getattr(self, "home_override", None)
        if ov:
            x, y = ov[0], ov[1]
            yaw = ov[2]
        qz = _m.sin(yaw / 2.0)
        qw = _m.cos(yaw / 2.0)
        self.stage = "going_home"
        self.home_target = (x, y)
        rospy.loginfo("[bridge] ★ 开始回起点 (%.2f, %.2f, yaw=%.1f°)", x, y, yaw * 180 / _m.pi)
        self.publish_goal((x, y, z, qz, qw), "起点")

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
        # ★ 回起点完成
        if getattr(self, "stage", None) == "going_home":
            rospy.loginfo("[bridge] ✅ 已回到起点，待命")
            self.stage = None
            self.report("idle", note="back_home")
            return
        with self.lock:
            stage = self.stage
        if stage == "nav_to_pick":
            rospy.loginfo("[bridge] 已到达取货点")
            self.report("arrived", task_id=self.task.get("task_id"),
                        site=self.task.get("site"))
            # ★★ 不立刻抓货！等平台确认"货物已入库"再发 pick 指令
            #     （车到了就安静等，货一到马上抓 —— 避免抓空气，也不用空跑）
            self.stage = "wait_pick_cmd"
            rospy.loginfo("[bridge] 已到位，等平台确认货物入库后下发 pick 指令…")
        elif stage == "nav_to_deliver":
            rospy.loginfo("[bridge] 已到达送达点")
            # ★★ 上报"到达目的地"
            self.report("dst_arrive", task_id=(self.task or {}).get("task_id"))
            self.stage = "arm_dropping"
            self.arm_seq(DROP_SEQ[:])          # grap → 松开夹爪 → 归位
        else:
            rospy.logwarn("[bridge] 到达但阶段未知（%s）", stage)

    # ---------- 夹爪 ----------
    def on_marker(self, msg):
        """★★★ 收到 AR 码识别结果 → 记下【序号(id)】+ 时间 + 距离

        ★ id 就是"这件货的身份证"：一件货贴一枚 AR 码，平台按序号认货 ✓
          小车抓取时把这个号上报，平台就知道"车上装的是哪一件"。
        """
        try:
            if msg.markers:
                self._marker_ts = time.time()
                m0 = msg.markers[0]
                p = m0.pose.pose.position
                self._marker_dist = (p.x * p.x + p.y * p.y + p.z * p.z) ** 0.5
                try:
                    self._marker_id = int(m0.id)
                except Exception:
                    self._marker_id = None
        except Exception:
            pass

    # ---------- ★ 只发姿态（不触发任何完成回调，给自定义序列用）----------
    def _pub_pose(self, name, keep_grip=False):
        """★ 发一个预设姿态。keep_grip=True 时【保留当前夹爪开合】，
           否则用该姿态预设的夹爪值（init/sentry/grap 预设都是 30=张开）。
           ⚠️ 抓完货回 init 时【必须】keep_grip=True，否则夹爪又被张开、
              货物会掉（这就是"识别到 AR 但夹不住"的原因）"""
        j = ARM_POSES.get(name)
        if j is None or self.pub_gripper is None:
            rospy.logerr("[bridge] 发姿态失败：%s", name)
            return False
        j = list(j)
        if keep_grip:
            j[5] = self.arm_angles[5]        # ★ 保持夹爪现状（别张开）
        self.arm_angles = list(j)
        self.holding = bool(j[5] > 90)     # ★ 按命令值推断"是否夹着"
        m = ArmJoint()
        m.joints = list(j)
        m.run_time = ARM_RUN_TIME
        self.pub_gripper.publish(m)
        rospy.loginfo("[bridge] 机械臂 → %s = %s%s（%.1f 秒）", name,
                      [int(x) for x in j], "（夹爪保持）" if keep_grip else "",
                      ARM_RUN_TIME / 1000.0)
        return True

    def _pub_grip(self, close):
        if self.pub_gripper is None:
            return False
        a = list(self.arm_angles)
        a[5] = 135.0 if close else 30.0
        m = ArmJoint()
        m.joints = a
        m.run_time = GRIP_RUN_TIME      # ★ 夹爪单独用更慢的速度
        self.pub_gripper.publish(m)
        # ★★★ 必须把新角度写回 self.arm_angles！
        #     否则后面 keep_grip=True 读到的还是旧值(30=张开) → 刚夹住又松开 ✗
        self.arm_angles = list(a)
        self.holding = bool(close)      # ★ 记住"正夹着货"，别被 joint_states 覆盖
        rospy.loginfo("[bridge] 夹爪 → %s (joints[5]=%.0f, %.1f 秒慢慢来)",
                      "闭合" if close else "张开", a[5], GRIP_RUN_TIME / 1000.0)
        return True

    def on_joint_state(self, msg):
        """同步 MoveIt 的关节状态（换算方式照抄 SimulationToMachine.cpp）"""
        if len(msg.position) == 5:
            for i in range(5):
                self.arm_angles[i] = (180 / 3.14) * msg.position[i] + 90
        elif len(msg.position) == 1:
            # ★ 正夹着货时，不让外部状态把夹爪改回张开（否则货当场就掉了）
            if getattr(self, "holding", False):
                return
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
        m.run_time = GRIP_RUN_TIME
        self.pub_gripper.publish(m)
        self.arm_angles = list(a)          # ★★ 写回（同 _pub_grip 的坑）
        self.holding = bool(close)
        rospy.loginfo("[bridge] 夹爪 → %s (joints[5]=%.0f, %.1f 秒慢慢来)",
                      "闭合" if close else "张开", a[5], GRIP_RUN_TIME / 1000.0)
        # TargetAngle 没有完成回执 → 等 2 秒当作完成
        threading.Timer(2.0, self._arm_done).start()

    # ---------- 机械臂 ----------
    def arm_named(self, name, keep_grip=False):
        """★ 直接发舵机角度到 /TargetAngle（绕开 MoveIt，不依赖 moveit_pose）
           所以【不需要起 bringup】，也就不会和原厂那套分拣状态机抢导航 ✓"""
        if self.dry:
            rospy.logwarn("[bridge·干跑] 本应让机械臂做姿态：%s", name)
            threading.Timer(2.5, self._arm_done).start()
            return
        j = ARM_POSES.get(name)
        if j is None:
            rospy.logwarn("[bridge] 未知姿态名 %s（可用：%s）", name, list(ARM_POSES.keys()))
            self._arm_done()
            return
        if self.pub_gripper is None:
            rospy.logerr("[bridge] 机械臂不可用（缺 ArmJoint 消息）")
            self._arm_done()
            return
        j = list(j)
        if keep_grip:
            j[5] = self.arm_angles[5]         # ★ 保持夹爪现状（别张开）
        self.arm_angles = list(j)             # 记住当前姿态（夹爪控制要用）
        self.holding = bool(j[5] > 90)        # ★ 按命令值推断"是否夹着"
        m = ArmJoint()
        m.joints = list(j)
        m.run_time = ARM_RUN_TIME
        self.pub_gripper.publish(m)
        rospy.loginfo("[bridge] 机械臂 → %s = %s%s（%.1f 秒）", name,
                      [int(x) for x in j], "（夹爪保持）" if keep_grip else "",
                      ARM_RUN_TIME / 1000.0)
        threading.Timer(2.5, self._arm_done).start()

    def arm_seq(self, seq):
        """依次执行姿态序列，最后收尾"""
        self._seq = list(seq)
        self._arm_seq_next()

    def _arm_seq_next(self):
        """执行动作序列的下一步；序列走完按当前阶段收尾"""
        if not self._seq:
            if self.stage == "arm_picking":
                rospy.loginfo("[bridge] 取货动作完成")
                # ★★ 上报"装载"（带运单号则精确匹配；不带给平台自动匹配）
                self.report("car_load", task_id=self.task.get("task_id"),
                            slot=self.task.get("slot"), count=self.task.get("count"),
                            marker_id=self._marker_id,      # ★★ 货物序号
                            tracking_no=self.task.get("tracking_no") or "")
                self.report("pick_done", task_id=self.task.get("task_id"),
                            slot=self.task.get("slot"), count=self.task.get("count"))
                self.stage = "wait_deliver"
            elif self.stage == "arm_dropping":
                rospy.loginfo("[bridge] 已放下，任务完成")
                # ★★ 上报"放入货柜"
                self.report("shelf_place", task_id=self.task.get("task_id"),
                            dst=(self.task or {}).get("dst") or "",
                            shelf_no=(self.task or {}).get("shelf_no") or "")
                self.report("delivered", task_id=self.task.get("task_id"))
                self.task = None
                self.report("idle")            # ★ 空闲了，可以接下一个任务
                # ★★★ 送完货【自动返回起始点】（不想要就把 AUTO_GO_HOME 设 0）
                if AUTO_GO_HOME:
                    self.go_home()
                else:
                    self.stage = None
                    rospy.loginfo("[bridge] 已关闭自动回位（AUTO_GO_HOME=0）")
            return
        step = self._seq.pop(0)
        if step == "__close__":
            self.set_gripper(True)
        elif step == "__open__":
            self.set_gripper(False)
        else:
            # ★★★ 只要夹爪【正夹着货】，任何姿态都不许把它张开
            #     否则：取货时刚夹住又张开、放货时一边下降一边松开（货在半空掉）
            #     只有显式 __open__ 才能松开 ✓
            keep = getattr(self, "holding", False)
            self.arm_named(step, keep_grip=keep)

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
