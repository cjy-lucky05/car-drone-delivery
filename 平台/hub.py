# -*- coding: utf-8 -*-
"""
平台程序（服务器端）
====================
职责：订阅所有设备消息 → 维护库位/任务/设备状态 → 下发指令 → 形成协同闭环

跑在哪：服务器上（连本地 127.0.0.1:1884，最快最稳）
用法：
    python3 platform.py                  # 前台运行（看日志）
    nohup python3 platform.py > platform.log 2>&1 &   # 后台运行

依赖：pip3 install paho-mqtt

核心协同逻辑（★ 这是本课题的"协同"所在）：
    1) 无人机投放前，向平台询问"我该放哪个库位"  → 平台按占用情况分配
    2) 无人机上报"投放完成" → 平台登记库位有货 → 生成取货任务 → 下发给对应小车
    3) 小车上报"取货完成" → 平台把库位置空 → 影响无人机下次投放的库位选择
    ★ 也就是：无人机的投放位置，取决于小车的取货状态（双向依赖）
"""

import json
import os
import signal
import sys
import time

# ★ 让脚本能找到装在 cjy 目录下的依赖（用 pip3 install --target=/root/chenjiayu/libs 装的）
_LOCAL_LIBS = "/root/chenjiayu/libs"
if os.path.isdir(_LOCAL_LIBS) and _LOCAL_LIBS not in sys.path:
    sys.path.insert(0, _LOCAL_LIBS)

# ★ 让日志实时输出（否则 nohup 重定向到文件时会被块缓冲，看不到输出）
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except Exception:
    pass

import paho.mqtt.client as mqtt

# ==================== 配置 ====================
# ---- 配置读取：优先 config_local.py，其次环境变量（★ 明文密码不进仓库）----
try:
    import config_local as _CFG
except ImportError:
    _CFG = None

def _cfg(key, default=""):
    v = getattr(_CFG, key, None) if _CFG else None
    return v if v not in (None, "") else os.getenv(key, default)

BROKER   = _cfg("MQTT_BROKER", "127.0.0.1")   # 平台跑在服务器上，连本地最稳
PORT     = int(_cfg("MQTT_PORT", "1884"))
USERNAME = _cfg("MQTT_USER_HUB", "cjy-car")   # 暂借小车账号（原 cjy-platform 认证有问题）
PASSWORD = _cfg("MQTT_PASS_HUB", "")
if not PASSWORD:
    print("[平台] ⚠️ 未读到密码！请复制 config.example.py 为 config_local.py 并填写 MQTT_PASS_HUB")
    sys.exit(1)

STATE_FILE = "/root/chenjiayu/platform_state.json"   # 状态持久化

# 站点 → 负责的小车（阶段 2 先用"固定分工"，后面升级成动态分配）
SITE_TO_CAR = {
    "1": "car-01",
    "2": "car-02",
    "3": "car-03",
}
SLOTS_PER_SITE = ["A", "B", "C"]      # 每个站点的库位编号


# ==================== 平台 ====================
class Platform:
    def __init__(self):
        # 库位：{"1A": {"site":"1","occupied":True,"count":1,"task_id":...,"ts":...}}
        self.slots = {}
        # 任务：{task_id: {"task_id","site","slot","count","car_id","status","ts"}}
        self.tasks = {}
        # 设备：{device_id: {"role","online","last_seen"}}
        self.devices = {}
        # ★ 已分配但未确认投放的记录 {device_id: {"site","slot","ts"}}
        self.pending_alloc = {}
        # ★ 待派发任务队列：按【站点】分开，各自 FIFO（先进库的先派）
        #   {"1": ["T0001", "T0003"], "2": [...], ...}
        self.queue = {}
        self.task_seq = 0
        self._load()

    # ---------- 状态持久化 ----------
    def _load(self):
        try:
            with open(STATE_FILE, encoding="utf-8") as f:
                d = json.load(f)
            self.slots = d.get("slots", {})
            self.tasks = d.get("tasks", {})
            self.queue = d.get("queue", {})
            self.task_seq = d.get("task_seq", 0)
            print(f"[平台] 已加载状态：库位 {len(self.slots)} 个，任务 {len(self.tasks)} 个")
        except FileNotFoundError:
            print("[平台] 首次启动，状态为空")

    def _save(self):
        try:
            with open(STATE_FILE, "w", encoding="utf-8") as f:
                json.dump({"slots": self.slots, "tasks": self.tasks,
                           "queue": self.queue, "task_seq": self.task_seq}, f,
                          ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"[平台] ⚠️ 状态保存失败：{e}")

    # ---------- 库位 ----------
    def alloc_slot(self, site):
        """
        分配库位（★ 环形队列 / FIFO 语义）

        为什么不能简单地"找第一个空位"：
            那样会在 A 被取空后立刻又往 A 放，导致小车永远在取 A，
            B/C 里的旧货被跳过、一直积压（饿死）。
        正确做法（本函数）：
            新货放到【最后一个有货位的下一个位置】，形成 A→B→C→A 的循环队列，
            保证"先放的先被取走"（FIFO），和固定的取货顺序一致。
        """
        sids = [f"{site}{s}" for s in SLOTS_PER_SITE]
        last_occ = -1
        for i, sid in enumerate(sids):
            if self.slots.get(sid, {}).get("occupied"):
                last_occ = i
        nxt = (last_occ + 1) % len(sids)
        sid = sids[nxt]
        if self.slots.get(sid, {}).get("occupied"):
            return None          # 所有库位都满了
        return sid

    def slot_occupied(self, sid, count, task_id):
        self.slots[sid] = {"site": sid[0], "occupied": True, "count": count,
                           "task_id": task_id, "ts": int(time.time())}

    def slot_clear(self, sid):
        if sid in self.slots:
            self.slots[sid].update({"occupied": False, "count": 0, "task_id": None,
                                    "ts": int(time.time())})

    # ---------- 任务 ----------
    def new_task(self, site, slot, count):
        self.task_seq += 1
        tid = f"T{self.task_seq:04d}"
        # ★ 不在这里定车：入队后由 dispatch() 派给【空闲】的车
        self.tasks[tid] = {"task_id": tid, "site": site, "slot": slot, "count": count,
                           "car_id": None, "status": "PENDING_PICK",
                           "created": int(time.time()), "updated": int(time.time())}
        return tid

    def set_task_status(self, tid, status):
        if tid in self.tasks:
            self.tasks[tid]["status"] = status
            self.tasks[tid]["updated"] = int(time.time())

    # ---------- 派单（★ 固定分工 + FIFO）----------
    def dispatch(self, client):
        """
        ★ 派单规则（固定分工）：
          · 站点1 的任务 → 只派给 1 号车；站点2 → 2 号车；以此类推
          · 每个站点一个队列，按【入库顺序】FIFO 出队（先进先出）
          · 车在忙 → 该站点的任务继续排队，等它空闲再派（一辆车一次只干一个）
        """
        changed = False
        for car_id, info in list(self.devices.items()):
            if info.get("role") != "car" or not info.get("online") or info.get("busy"):
                continue
            # ★ 这辆车负责哪些站点（固定分工）
            my_sites = [s for s, c in SITE_TO_CAR.items() if c == car_id]
            for site in my_sites:
                q = self.queue.get(site) or []
                if not q:
                    continue
                tid = q.pop(0)                      # ★ 队头出队 = 先进先出
                t = self.tasks.get(tid)
                if not t or t["status"] != "PENDING_PICK":
                    changed = True                  # 清掉无效项也要存盘
                    continue
                t["car_id"] = car_id
                t["status"] = "DISPATCHED"
                info["busy"] = True
                print(f"[平台] 派单 {tid}（站点{site} 库位 {t['slot']}）→ {car_id}"
                      f"（该站点队列还剩 {len(q)} 个）")
                self.pub_cmd(client, car_id, {"cmd": "goto_pick", "task_id": tid,
                                              "site": t["site"], "slot": t["slot"],
                                              "count": t["count"]})
                changed = True
                break                                # ★ 一辆车一次只接一个任务
        if changed:
            self._save()
            self.dump_status()

    def pub_cmd(self, client, device_id, payload):
        topic = f"cjy/{device_id}/cmd"
        client.publish(topic, json.dumps(payload, ensure_ascii=False))
        print(f"[平台] → 下发 {topic}: {json.dumps(payload, ensure_ascii=False)}")

    def broadcast(self, client, payload):
        client.publish("cjy/broadcast", json.dumps(payload, ensure_ascii=False))
        print(f"[平台] → 广播: {json.dumps(payload, ensure_ascii=False)}")

    def dump_status(self):
        """打印当前状态（方便观察闭环）"""
        print("─" * 60)
        print(f"[状态] 库位：", end="")
        for sid in sorted(self.slots):
            s = self.slots[sid]
            mark = "有货" if s.get("occupied") else "空"
            print(f"{sid}={mark}({s.get('count', 0)}) ", end="")
        print()
        pending = [t for t in self.tasks.values() if t["status"] != "DONE"]
        print(f"[状态] 未完成任务 {len(pending)} 个：" +
              (", ".join(f"{t['task_id']}@{t['slot']}[{t['status']}]" for t in pending) or "无"))
        qs = "，".join(f"站点{s}排队{len(v)}个" for s, v in sorted(self.queue.items()) if v)
        print(f"[状态] 派单队列：{qs or '空'}")
        print("─" * 60)

    # ---------- 消息处理 ----------
    def handle_report(self, client, device_id, data):
        role = data.get("role") or data.get("device")
        # ★ 保留 busy 状态（设备重连不清空）
        info = self.devices.setdefault(device_id, {"role": role, "busy": False})
        info.update({"role": role, "online": True, "last_seen": int(time.time())})
        event = data.get("event")
        if event == "selftest":
            return                      # 设备自检事件，忽略

        # ★ 设备声明"我空闲了" → 立即派下一个任务（拉模型：车空闲时才派）
        if event == "idle":
            self.devices[device_id]["busy"] = False
            print(f"[平台] {device_id} 空闲，检查是否有待派任务")
            self.dispatch(client)
            return

        # ===== 无人机端 =====
        if event == "ask_slot":
            # 无人机问：我该放哪个库位？→ 平台按占用情况分配（★ 协同点）
            site = str(data.get("site"))
            sid = self.alloc_slot(site)
            if sid is None:
                print(f"[平台] ⚠️ 站点 {site} 所有库位已满，通知无人机等待")
                self.pub_cmd(client, device_id, {"cmd": "wait", "site": site,
                                                 "reason": "库位已满"})
            else:
                # ★ 记录平台的分配结果，投放确认时以此为准（不信任设备上报的 slot）
                self.pending_alloc[device_id] = {"site": site, "slot": sid,
                                                 "ts": int(time.time())}
                self.pub_cmd(client, device_id, {"cmd": "use_slot", "site": site, "slot": sid})
            return

        if event == "drop_done":
            # 投放完成 → 登记库位 → 生成取货任务 → 通知小车（★ 协同点）
            site = str(data.get("site"))
            # ★ 以平台分配的结果为准（人不该覆盖库位分配）
            alloc = self.pending_alloc.pop(device_id, None)
            if alloc and alloc.get("site") == site:
                sid = alloc["slot"]
                if data.get("slot") and data["slot"] != sid:
                    print(f"[平台] ⚠️ {device_id} 上报库位 {data['slot']} 与平台分配 {sid} 不一致 → 以平台为准")
            else:
                sid = data.get("slot") or f"{site}A"     # 没问过库位就按上报的（兜底）
            if self.slots.get(sid, {}).get("occupied"):
                print(f"[平台] ⚠️ 库位 {sid} 已被占用，拒绝重复投放（本次忽略）")
                return
            count = int(data.get("count", 1))
            tid = self.new_task(site, sid, count)
            self.slot_occupied(sid, count, tid)
            q = self.queue.setdefault(site, [])     # ★ 加入【该站点】的队列
            q.append(tid)
            print(f"[平台] {device_id} 投放到 {sid}（{count} 件）→ 任务 {tid} 加入站点{site}队列"
                  f"（排队 {len(q)} 个，等待 {SITE_TO_CAR.get(site)} 取货）")
            self.dispatch(client)           # 有空闲车就派
            self._save()
            self.dump_status()
            return

        if event == "depart":
            print(f"[平台] 无人机 {device_id} 起飞/离站：{data.get('site')}")
            return

        # ===== 小车端 =====
        if event == "arrived":
            tid = data.get("task_id")
            self.set_task_status(tid, "PICKING")
            print(f"[平台] 小车 {device_id} 已到达 {data.get('site')}，任务 {tid} 开始取货")
            self._save()
            return

        if event == "pick_done":
            # 取货完成 → 库位置空（★ 影响无人机下次投放）→ 通知小车去送达
            tid = data.get("task_id")
            task = self.tasks.get(tid)
            if task:
                self.slot_clear(task["slot"])
                self.set_task_status(tid, "DELIVERING")
                print(f"[平台] 小车 {device_id} 已取走 {task['slot']}（{task['count']} 件）"
                      f"→ 库位置空，任务 {tid} 进入配送")
                self.pub_cmd(client, device_id, {"cmd": "deliver", "task_id": tid,
                                                 "count": task["count"]})
                self.broadcast(client, {"type": "slot_update", "slot": task["slot"],
                                        "occupied": False})
            self._save()
            self.dump_status()
            return

        if event == "delivered":
            tid = data.get("task_id")
            self.set_task_status(tid, "DONE")
            self.devices[device_id]["busy"] = False     # ★ 车空闲了
            print(f"[平台] 小车 {device_id} 已送达，任务 {tid} 完成 ✓")
            self._save()
            self.dispatch(client)                       # ★ 立刻派下一个任务
            self.dump_status()
            return

        if event == "position":
            # 位置上报（高频，不打日志，只更新设备状态）
            return

        if event == "exception":
            print(f"[平台] ⚠️ 设备 {device_id} 上报异常：{data.get('reason')}（任务 {data.get('task_id')}）")
            return

        print(f"[平台] ? 未知事件 {event}，来自 {device_id}：{data}")


# ==================== MQTT 接入 ====================
def main():
    if "待填" in PASSWORD:
        print("⚠️ 平台账号密码还没填！请先建 cjy-platform 账号（见文末），再把密码写进 PASSWORD")
        print("   临时方案：把 USERNAME/PASSWORD 改成 cjy-car / 你的小车密码，也能跑")
        sys.exit(1)

    plat = Platform()
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="cjy-platform")

    def on_connect(c, u, f, rc, p=None):
        if rc == 0:
            print(f"[平台] ✅ 已连接 {BROKER}:{PORT}")
            c.subscribe("cjy/#")            # 订阅所有设备消息
            print("[平台] 已订阅 cjy/#（所有设备）")
            plat.dump_status()
        else:
            print(f"[平台] ❌ 连接失败 rc={rc}（密码错？）")

    def on_message(c, u, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace")
        parts = topic.split("/")
        # cjy/{device_id}/report
        if len(parts) >= 3 and parts[0] == "cjy":
            device_id = parts[1]
            kind = parts[2]
            if kind == "report":
                try:
                    data = json.loads(payload)
                except Exception:
                    print(f"[平台] ⚠️ 非法 JSON：{topic} {payload}")
                    return
                plat.handle_report(c, device_id, data)
                plat.dispatch(c)        # ★ 每次收到上报都尝试派单（只派给空闲车）

    def on_disconnect(c, u, flags=None, rc=None, p=None):
        print(f"[平台] 连接断开 rc={rc}，自动重连")

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.username_pw_set(USERNAME, PASSWORD)
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    try:
        client.connect(BROKER, PORT, keepalive=60)
    except Exception as e:
        print(f"[平台] 连不上 {BROKER}:{PORT} → {e}")
        sys.exit(1)

    # Ctrl+C 优雅退出（顺手存一次状态）
    def bye(sig, frame):
        print("\n[平台] 收到退出信号，保存状态…")
        plat._save()
        client.disconnect()
        sys.exit(0)
    signal.signal(signal.SIGINT, bye)

    print("[平台] 运行中，Ctrl+C 退出")
    client.loop_forever()


if __name__ == "__main__":
    main()
