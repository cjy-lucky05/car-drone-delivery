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
import random
import signal
import sys
import threading
import time

# ★ 让脚本能找到装在 cjy 目录下的依赖（用 pip3 install --target=/root/chenjiayu/libs 装的）
_LOCAL_LIBS = "/root/chenjiayu/libs"
if os.path.isdir(_LOCAL_LIBS) and _LOCAL_LIBS not in sys.path:
    sys.path.insert(0, _LOCAL_LIBS)

# ★ 让脚本找到 web平台/server/db.py（数据库访问层）
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in ("/root/chenjiayu/web平台/server",
           os.path.join(_HERE, "..", "web平台", "server"),
           os.path.join(_HERE, "web平台", "server")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

# ★ 让日志实时输出（否则 nohup 重定向到文件时会被块缓冲，看不到输出）
try:
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
except Exception:
    pass

import paho.mqtt.client as mqtt

# ★ 数据库访问层（web平台/server/db.py）—— 状态持久化改用 SQLite
try:
    import db as _db
except ImportError:
    print("[平台] ⚠️ 找不到 db.py！请确认 web平台/server/ 在 sys.path 里")
    raise

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

STATE_FILE = "/root/chenjiayu/platform_state.json"   # （旧）JSON 状态文件 —— 已改用 SQLite
# ★ 数据库文件位置：环境变量 DELIVERY_DB 优先，默认 web平台/data/delivery.db

# 站点 → 负责的小车（阶段 2 先用"固定分工"，后面升级成动态分配）
SITE_TO_CAR = {
    "1": "car-01",
    "2": "car-02",
    "3": "car-03",
}
SLOTS_PER_SITE = ["A", "B", "C"]      # 每个站点的库位编号

# ★★★ 包裹（2026-10-08 新增）—— 解决"物流到驿站就断了、校内没信息"的痛点
PARCEL_STATUS = ("CREATED", "ON_DRONE", "AT_GATE", "LOADED",
                 "DELIVERING", "PLACED", "PICKED")
PARCEL_CN = {
    "CREATED": "已揽收", "ON_DRONE": "无人机运输中", "AT_GATE": "已到校门口站点",
    "LOADED": "小车已装载", "DELIVERING": "校园配送中",
    "PLACED": "已放入货柜(待取)", "PICKED": "已取件",
}
PARCEL_EV_CN = {
    "parcel_in":    "中转点揽收挂载",
    "drone_depart": "无人机起飞",
    "drone_arrive": "无人机到达校门口站点",
    "car_load":     "小车装载（扫码识别）",
    "car_depart":   "小车出发配送",
    "dst_arrive":   "到达目的地",
    "shelf_place":  "放入货柜(低层)",
    "picked_up":    "扫码取件（已签收）",
    # ★★★ 2026-10-09 补全（之前漏了这些，时间线上直接显示英文 ✗）
    "set_dest":     "修改送达地点",
    "parcel_add":   "手动添加包裹",
    "parcel_del":   "删除包裹",
    "drop_done":    "已放货到校门口站点",
    "drone_takeoff":"无人机起飞",
    "drone_loaded": "已取货（挂载）",
    "takeoff":      "无人机起飞",
    "loaded":       "已取货（挂载）",
    "ask_slot":     "询问库位",
    "use_slot":     "平台分配库位",
    "wait":         "原地等待",
    "pick":         "开始抓取",
    "goto_pick":    "前往取货点",
    "goto_deliver": "前往送达点",
    "arrived":      "到达取货点",
    "pick_done":    "取货完成",
    "delivered":    "送达完成",
    "go_home":      "返回起始点",
    "back_home":    "已回到起点",
    "idle":         "空闲待命",
    "busy":         "任务执行中",
    "exception":    "设备异常",
    "selftest":     "设备自检",
    "heartbeat":    "心跳保活",
    "position":     "位置上报",
    "gps":          "GPS位置上报",
    "battery":      "电量上报",
}


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
        # ★ 已在取货点【等待货物入库】的车：{device_id: {"task_id","site","slot"}}
        #   无人机投放完成后，据此立刻通知它开抓
        self.waiting_car = {}
        self.task_seq = 0
        self.parcel_seq = 0          # ★ 包裹编号游标
        # ★★ 哪台车当前装着哪件货：{device_id: parcel_id}
        #    小车端上报"出发/到达/放货柜"时可能不带包裹号 → 用这个自动对应
        self.car_parcel = {}
        self._load()

    # ---------- 状态持久化 ----------
    def _precreate_slots(self):
        """★ 启动时把各站点的库位先建出来（空着也显示）→ 网页一打开就有货架"""
        created = 0
        for site in sorted(SITE_TO_CAR):
            for suf in SLOTS_PER_SITE:
                sid = f"{site}{suf}"
                if sid not in self.slots:
                    self.slots[sid] = {"site": site, "occupied": False, "count": 0,
                                       "task_id": None, "ts": int(time.time())}
                    created += 1
        if created:
            self._save()
            print(f"[平台] 已预建 {created} 个库位（{len(SITE_TO_CAR)} 站点 × {len(SLOTS_PER_SITE)} 位）")

    def _load(self):
        """★ 从 SQLite 数据库重建内存状态（取代原来的 JSON 文件）"""
        try:
            _db.init_db()                       # 建表（幂等）
            for r in _db.list_slots():
                self.slots[r["slot_id"]] = {
                    "site": r["site"], "occupied": bool(r["occupied"]),
                    "count": r["count"] or 0, "task_id": r["task_id"],
                    "ts": r["updated_at"] or 0}
            for r in _db.list_tasks(limit=100000):
                self.tasks[r["task_id"]] = {
                    "task_id": r["task_id"], "site": r["site"], "slot": r["slot"],
                    "count": r["count"], "car_id": r["car_id"], "status": r["status"],
                    # ★★★ 目的地 + 要取哪一号货：必须恢复，否则重启后这趟任务"不知道要取哪件"✗
                    "dst": (r["dst"] if "dst" in r.keys() else "") or "",
                    "marker_id": (r["marker_id"] if "marker_id" in r.keys() else None),
                    "created": r["created_at"] or 0, "updated": r["updated_at"] or 0}
                try:
                    self.task_seq = max(self.task_seq, int(str(r["task_id"]).lstrip("T")))
                except Exception:
                    pass
            for r in _db.list_queue():          # 按 id 升序 = 保持 FIFO 顺序
                self.queue.setdefault(r["site"], []).append(r["task_id"])
            for r in _db.list_devices():
                self.devices[r["device_id"]] = {
                    "role": r["role"], "online": False,
                    "busy": (r["status"] == "busy"), "last_seen": r["last_seen"] or 0}
            for r in _db.list_parcels(limit=100000):      # ★ 恢复包裹编号游标
                try:
                    self.parcel_seq = max(self.parcel_seq,
                                          int(str(r["parcel_id"]).lstrip("P")))
                except Exception:
                    pass
            print(f"[平台] 已从数据库加载：库位 {len(self.slots)} 个，任务 {len(self.tasks)} 个，"
                  f"设备 {len(self.devices)} 个，包裹 {self.parcel_seq} 件")
            self._precreate_slots()             # ★ 预建库位，网页直接能看到货架
        except Exception as e:
            print(f"[平台] ⚠️ 数据库加载失败（将以空状态启动）：{e}")

    def _save(self):
        """★ 把内存状态写回 SQLite（库位 + 任务 + 队列 + 设备）"""
        try:
            for sid, s in self.slots.items():
                _db.upsert_slot(sid, s.get("site", sid[0]), bool(s.get("occupied")),
                                int(s.get("count", 0) or 0), s.get("task_id"))
            for tid, t in self.tasks.items():
                _db.upsert_task(tid, t.get("site", ""), t.get("slot", ""),
                                int(t.get("count", 0) or 0), t.get("status", "PENDING_PICK"),
                                t.get("car_id"),
                                dst=t.get("dst") or "",          # ★ 目的地落库
                                marker_id=t.get("marker_id"))    # ★ 要取哪一号落库
            items = []
            for site, tids in self.queue.items():
                for tid in tids:
                    items.append((site, tid))
            _db.replace_queue(items)
            for did, d in self.devices.items():
                _db.upsert_device(did, d.get("role", ""), "busy" if d.get("busy") else "idle")
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
    def new_task(self, site, slot, count, dst="", marker_id=None):
        """★★★ 建任务。dst = 【派单时锁定的目的地】；marker_id = 【AR码序号】

        - dst：目的地一旦派给小车的任务，就【不再允许中途改】✗
        - marker_id：这件货的"身份证"。小车抓取时用摄像头读 AR 码得到序号，
          平台据此确认"车上装的是哪一件"（也用于校验有没有取错件）✓
        """
        self.task_seq += 1
        tid = f"T{self.task_seq:04d}"
        self.tasks[tid] = {"task_id": tid, "site": site, "slot": slot, "count": count,
                           "car_id": None, "status": "PENDING_PICK", "dst": dst or "",
                           "marker_id": marker_id,
                           "created": int(time.time()), "updated": int(time.time())}
        return tid

    def set_task_status(self, tid, status):
        if tid in self.tasks:
            self.tasks[tid]["status"] = status
            self.tasks[tid]["updated"] = int(time.time())

    # ---------- 派单（★ 固定分工 + FIFO）----------
    def patrol(self, client):
        """★★★ 库位巡检（2026-10-08 新增）

        解决：库位里【已经有货】但平台不知道要去取的情况。
        以前只有收到 drop_done 才生成任务；如果货是【上一批留下】的、
        或无人机没发指令，库位就一直压着，车永远不去取。

        逻辑：扫一遍所有库位 → 有货 且 没有关联任务 → 补生成任务 + 入队派车
        """
        n = 0
        for sid, sl in list(self.slots.items()):
            if not sl.get("occupied"):
                continue
            if not int(sl.get("count") or 0):
                continue
            if sl.get("task_id"):
                continue                       # 已经有任务在跟，跳过
            site = str(sl.get("site") or (sid[0] if sid else "1"))
            count = int(sl.get("count") or 1)

            # ★★★ 从【该货格里的那件货】捞出"送到哪 + 要取哪一号"
            #    巡检补的任务也必须带这两个 —— 否则派单下去车"不知道取哪件、送哪"✗
            _dst, _mk = "", None
            try:
                for _p in _db.list_parcels(limit=500):
                    if str(_p.get("shelf_no") or "").strip().upper() != str(sid).upper():
                        continue
                    if str(_p.get("status") or "").upper() not in ("AT_GATE", "PLACED", "CREATED"):
                        continue
                    _dst = _p.get("dst") or ""
                    _mk = _p.get("marker_id")
                    break
            except Exception:
                pass

            tid = self.new_task(site, sid, count, dst=_dst, marker_id=_mk)
            self.slot_occupied(sid, count, tid)
            self.queue.setdefault(site, []).append(tid)
            print(f"[平台] 🔍 巡检：库位 {sid} 有货 {count} 件但没有任务 → 补生成 {tid}"
                  f"（站点{site}队列，等 {SITE_TO_CAR.get(site)} 来取）")
            n += 1
        # ★★★ 每次巡检都自愈 + 重新派单（不管这次有没有补新任务）
        #     这样"车卡 busy""队列压着任务"这类情况最多 5 秒就能自己恢复 ✓
        healed = self.heal_stale_busy(client)
        self.dispatch(client)
        if n or healed:
            self._save()
            self.dump_status()
        return n

    def sync_slots(self):
        """★★★ 以【包裹表】为准，重算每个货架的占位状态（2026-10-08）

        解决：货架上明明没货了，网页还显示"在库1件"（占位标记没跟着清）。
        规则：某货架格的【未取件】包裹数 > 0 → 占用；= 0 → 空。
        每次巡检都跑一遍，保证网页看到的和包裹表一致 ✓
        """
        try:
            # ★★★ 只有【真在货架上】的才算，三种：
            #     AT_GATE  = 在【校门口站点货架】(1A/1B…) 等小车取
            #     PLACED   = 已放入【配送点货柜】(2号公寓楼下-A…) 等收件人取
            #     CREATED  = 刚在中转点揽收
            on_shelf = ("AT_GATE", "PLACED", "CREATED")
            cnt = {}
            for p in _db.list_parcels(limit=2000):
                sn = str(p.get("shelf_no") or "").strip().upper()
                st = str(p.get("status") or "").upper()
                if not sn:
                    continue
                if st not in on_shelf:
                    continue                    # ★ 在途/已装车/已送达/已取件 → 不在货架上
                cnt[sn] = cnt.get(sn, 0) + 1
            changed = 0
            for slot in _db.list_slots():
                sid = str(slot.get("slot_id") or "").upper()
                if not sid:
                    continue
                n = cnt.get(sid, 0)
                was_occ = bool(slot.get("occupied"))
                was_cnt = int(slot.get("count") or 0)

                # ★★★ 关键：平台自己的【内存 self.slots】也要一起纠正 ✗
                #   平台"这个库位能不能投放"用的是内存，不是数据库。
                #   如果只同步数据库 → 外部改过库（手动 SQL / 网页删包裹）之后，
                #   内存还留着旧状态 → drop_done 会一直报"库位已被占用"拒绝投放 ✗
                mem = self.slots.setdefault(sid, {"site": sid[0]})
                mem_occ = bool(mem.get("occupied"))
                mem_cnt = int(mem.get("count") or 0)

                if (mem_occ != (n > 0)) or (mem_cnt != n) or (was_occ != (n > 0)) or (was_cnt != n):
                    _db.upsert_slot(sid, sid[0], n > 0, n)
                    mem.update({"site": sid[0], "occupied": (n > 0), "count": n,
                                "ts": int(time.time())})
                    if n <= 0:
                        mem["task_id"] = None      # 空了就解除任务关联
                    print(f"[平台] 🔄 货架 {sid}：{'占用' if was_occ else '空'} → "
                          f"{'占用' if n > 0 else '空'}（{n} 件）")
                    changed += 1
            return changed
        except Exception as e:
            print(f"[平台] ⚠️ 同步货架失败：{e}")
            return 0

    def start_patrol(self, client, interval=5.0):
        """★ 启后台巡检线程（每 interval 秒扫一次库位）"""
        def _loop():
            time.sleep(3)                      # 等 MQTT 连稳、状态加载完
            while True:
                try:
                    self.sync_slots()          # ★ 先让货架状态跟着包裹表走
                    self.patrol(client)        # ★ 再把"有货无任务"补成任务
                except Exception as e:
                    print(f"[平台] ⚠️ 巡检异常：{e}")
                time.sleep(interval)

        threading.Thread(target=_loop, daemon=True).start()
        print(f"[平台] ★ 库位巡检已启动（每 {interval:.0f} 秒一次：有货但无任务 → 自动派车）")

    def heal_stale_busy(self, client):
        """★★★ 自愈：车/机卡在 busy，但手头根本没有进行中的任务 → 复位空闲

        典型场景（真实踩过）：
          · 平台跑出多个进程抢消息 → 设备状态写乱
          · 平台重启后从库里读到脏的 busy 标记
        后果：车明明在待命，却永远接不到新任务（队列一直排队、车一动不动）。
        判定标准：这个设备名下【没有任何处于进行中的任务】→ 它就是空闲的。
        """
        ACTIVE = ("DISPATCHED", "PICKING", "LOADING", "DELIVERING", "GOING_HOME")
        healed = 0
        for dev_id, info in list(self.devices.items()):
            if not info.get("busy"):
                continue
            cur = None
            for t in self.tasks.values():
                if t.get("car_id") == dev_id and (t.get("status") or "") in ACTIVE:
                    cur = t
                    break
            if cur is not None:
                continue
            info["busy"] = False
            try:
                _db.upsert_device(dev_id, info.get("role", ""), "idle")
            except Exception:
                pass
            print(f"[平台] 🔧 自愈：{dev_id} 卡在 busy 但没有进行中的任务 → 复位为空闲")
            healed += 1
        return healed

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
                info["busy"] = True
                # ★★★ 派单日志：把【要取哪一号货 / 送到哪】打出来（便于追溯 + 论文截图）
                print(f"[平台] 派单 {tid}（站点{t['site']} 库位 {t['slot']}，"
                      f"AR码 {t.get('marker_id')}，送到 {t.get('dst') or '未指定'}）→ {car_id}")
                try:
                    _db.update_task(tid, assigned_at=int(time.time()))   # ★ 派单时间
                except Exception:
                    pass
                # ★★ 关键优化：这辆车是不是【已经在取货点等着了】？
                #    是 → 直接让它抓（省一趟来回导航）；否 → 正常派导航
                w = self.waiting_car.get(car_id)
                if w and w.get("site") == site:
                    self.waiting_car.pop(car_id, None)
                    t["status"] = "PICKING"
                    print(f"[平台] 车 {car_id} 已在站点{site}取货点等着 → 直接派抓取 {tid}"
                          f"（省一趟导航）")
                    self.pub_cmd(client, car_id, {"cmd": "pick", "task_id": tid,
                                                  "slot": t["slot"]})
                else:
                    t["status"] = "DISPATCHED"
                    print(f"[平台] 派单 {tid}（站点{site} 库位 {t['slot']}）→ {car_id}"
                          f"（该站点队列还剩 {len(q)} 个）")
                    self.pub_cmd(client, car_id, {"cmd": "goto_pick", "task_id": tid,
                                                  "site": t["site"], "slot": t["slot"],
                                                  "count": t["count"],
                                                  "dst": t.get("dst") or "",
                                                  "marker_id": t.get("marker_id")})
                changed = True
                break                                # ★ 一辆车一次只接一个任务
        if changed:
            self._save()
            self.dump_status()

    # ---------- ★★★ 包裹 ----------
    def new_parcel(self, tracking_no="", order_no="", owner="", phone="",
                   dst="", mode="SHELF", drone_id="", shelf_no=""):
        """建一件包裹：生成 P 编号 + 4 位取件码，返回 parcel_id"""
        self.parcel_seq += 1
        pid = "P%04d" % self.parcel_seq
        used = {p.get("pick_code") for p in _db.list_parcels(limit=500)}
        code = "%04d" % random.randint(1000, 9999)
        for _ in range(50):
            if code not in used:
                break
            code = "%04d" % random.randint(1000, 9999)
        # ★★ 调用方给了库位就用它（无人机投放的实际库位，如 1C）
        # ★★ 用户自己设过收货点 → 没指定目的地时用它（"送到哪由用户决定"）
        if not dst and phone:
            try:
                _u = _db.get_user_dst(phone)
                if _u:
                    dst = _u
                    print(f"[平台] ★ 按用户 {owner or phone} 的收货点设置目的地：{dst}")
            except Exception:
                pass
        shelf = shelf_no or (("%s-第1层-%d格" % (dst or "货柜", (self.parcel_seq % 8) + 1))
                             if mode == "SHELF" else "")
        _db.upsert_parcel(pid, tracking_no=tracking_no, order_no=order_no,
                          owner=owner, phone=phone, dst=dst, mode=mode,
                          shelf_no=shelf, pick_code=code, status="CREATED",
                          drone_id=drone_id)
        print("[平台] \u2605 新包裹 %s | %s | %s | 单号 %s | 取件码 %s | %s"
              % (pid, owner or "-", dst or "-", tracking_no or "-", code, shelf or "(送驿站)"))
        return pid

    def parcel_timeline(self, pid, event, detail="", actor=""):
        """给某件包裹记一条时间线事件（前端"按包裹看轨迹"靠这个）"""
        try:
            _db.log_event(actor or "platform", event,
                          {"detail": detail or PARCEL_EV_CN.get(event, event),
                           "parcel": pid}, parcel_id=pid)
        except Exception:
            pass

    def set_parcel(self, pid, status=None, event=None, detail="", actor="", **kw):
        """改包裹状态 + 记时间线（一步完成）"""
        if status:
            kw["status"] = status
        if kw:
            try:
                _db.update_parcel(pid, **kw)
            except Exception:
                pass
        if event:
            self.parcel_timeline(pid, event, detail, actor)

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
        try:
            ps = _db.parcel_stats()
            if ps.get("parcels_total"):
                print("[状态] 包裹：共 %d 件，待取 %d 件，已取 %d 件"
                      % (ps["parcels_total"], ps.get("parcels_waiting", 0),
                         ps.get("parcels_picked", 0)))
        except Exception:
            pass
        print("─" * 60)

    # ---------- 消息处理 ----------
    def handle_report(self, client, device_id, data):
        # ★★ 2026-10-08：人（收件人）不是设备！事件里的 actor 可能写成 "user"/"人"
        #    这类上报只记事件，【不登记为设备】—— 否则设备列表里会冒出个 user
        if str(device_id).lower() in ("user", "human", "person", "ren", "人") or \
           str(data.get("role") or "").lower() in ("user", "human", "person", "人"):
            ev = data.get("event")
            try:
                _db.log_event(device_id, ev or "unknown", data,
                              parcel_id=data.get("parcel_id") or None)
            except Exception:
                pass
            print(f"[平台] （人/用户操作）{ev}：{data.get('detail') or data.get('by') or ''}")
            return
        role = data.get("role") or data.get("device")
        # ★ 保留 busy 状态（设备重连不清空）
        info = self.devices.setdefault(device_id, {"role": role, "busy": False})

        # ★★★ 2026-10-09：区分【设备真上报】和【网页替设备发的上报】
        #   人在网页点"已放货"→ 平台替无人机发一条 report（from=web）。
        #   这条【不能算设备活着】✗ 否则点一下，离线的无人机就变"在线"了。
        #   在线 = 机载程序(NX)在上报；NX 在跑才代表飞机通电、可以操作 ✓
        _from_web = (str(data.get("from") or "").lower() == "web")
        if _from_web:
            info.setdefault("role", role)
        else:
            info.update({"role": role, "online": True, "last_seen": int(time.time())})
        event = data.get("event")
        # ★★★ 只把【有业务含义】的事件记进事件表
        #    位置/电量/GPS/心跳/自检 只是保活或画图用的高频上报，
        #    以前全记 → "事件记录"被心跳刷屏，真正的投递/装载/送达反而看不见 ✗
        _NOISY = ("position", "battery", "gps", "heartbeat", "selftest")
        _is_hb = (str(data.get("note") or "").lower() == "heartbeat")
        if (event not in _NOISY) and (not _is_hb):
            try:
                _db.log_event(device_id, event or "unknown", data)
            except Exception:
                pass
        # ★ 设备状态立即落库 —— 否则网页看不到"在线设备"（idle 分支下面会 return）
        #   ★★ 网页替发的上报：只保证设备"在册"，【不刷新在线时间】✗
        try:
            if _from_web:
                _db.register_device(device_id, info.get("role") or role or "drone", "")
            else:
                _db.upsert_device(device_id, info.get("role", ""),
                                  "busy" if info.get("busy") else "idle")
        except Exception:
            pass
        if event == "selftest":
            return                      # 设备自检事件，忽略

        # ★ 设备声明"我空闲了" → 立即派下一个任务（拉模型：车空闲时才派）
        if event == "idle":
            self.devices[device_id]["busy"] = False
            print(f"[平台] {device_id} 空闲，检查是否有待派任务")
            try:
                _db.upsert_device(device_id, info.get("role", ""), "idle")
            except Exception:
                pass
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
            # ★★ 目的地锁进任务（优先用无人机端填的 dst，其次走用户自己设的收货点）
            _dst = str(data.get("dst") or "")
            if not _dst:
                try:
                    _dst = _db.get_user_dst(str(data.get("phone") or "")) or ""
                except Exception:
                    _dst = ""
            # ★★★ AR 码序号：人在网页填了就用他的，没填平台自动分配一个空闲号
            _mk = data.get("marker_id")
            try:
                _mk = int(_mk) if _mk not in (None, "", "None") else None
            except Exception:
                _mk = None
            if _mk is None:
                try:
                    _mk = _db.next_free_marker()
                    print(f"[平台] 未填 AR 码序号 → 平台自动分配 {_mk} 号")
                except Exception:
                    _mk = None
            tid = self.new_task(site, sid, count, dst=_dst, marker_id=_mk)
            print(f"[平台] 任务 {tid} 目的地锁定：{_dst or '（未指定）'}；AR码序号 {_mk}")

            # ★★★ 关键：登记一件【包裹】
            #   以前只生成任务不建包裹 → 货架同步(按 parcels 表算)会把这个库位
            #   当成"空" → 小车到了就收到 wait"货物尚未入库" → 死等
            pid = None
            try:
                pid = self.new_parcel(
                    tracking_no=str(data.get("tracking_no") or ""),
                    owner=str(data.get("owner") or ""),
                    phone=str(data.get("phone") or ""),
                    dst=str(data.get("dst") or ""),
                    mode="SHELF", drone_id=device_id, shelf_no=sid)
                # ★★★ 状态必须是 AT_GATE（已到校门口站点，等小车来取）
                #     以前写成 PLACED（=已放入货柜待取，那是【送达后】的最终状态）
                #     → 货刚到站就被判"锁定"，用户再也不能改目的地 ✗
                _db.update_parcel(pid, shelf_no=sid, status="AT_GATE", drone_id=device_id,
                                  marker_id=_mk)
                self.tasks[tid]["parcel_id"] = pid
                try:
                    self.parcel_timeline(pid, "parcel_in",
                                         "无人机投放到库位 %s" % sid, device_id)
                except Exception:
                    pass
                _who = str(data.get("owner") or "").strip() or "未知收件人"
                print(f"[平台] ★ 已登记包裹 {pid}（{_who}，AR码 {_mk} 号）"
                      f"→ 库位 {sid}（谁的货到校门口了 ✓ 等小车来取）")
            except Exception as e:
                print(f"[平台] ⚠️ 登记包裹失败：{e}")

            self.slot_occupied(sid, count, tid)
            q = self.queue.setdefault(site, [])     # ★ 加入【该站点】的队列
            q.append(tid)
            # 注：这里【不】直接唤醒等待的车 —— 统一交给 dispatch() 处理
            #     （否则会出现"唤醒 + 派单"重复下指令；dispatch 会判断车是否已在取货点）
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
            task = self.tasks.get(tid) or {}
            sid = task.get("slot")
            if tid:
                print(f"[平台] 小车 {device_id} 已到达 {data.get('site')}，任务 {tid} 到位")
            else:
                print(f"[平台] 小车 {device_id} 已到达站点{data.get('site')}"
                      f"（手动指令、未关联任务）—— 先让它原地待命")
            if not sid:
                # 没有关联任务/库位（手动指令）→ 直接让等，不报"库位 None 暂无货"
                self.waiting_car[device_id] = {"task_id": None,
                                               "site": str(data.get("site") or ""),
                                               "slot": None}
                self.pub_cmd(client, device_id, {"cmd": "wait",
                                                 "reason": "手动指令，原地待命（等平台派活）"})
                self._save()
                return
            # ★★ 关键：到点先看【库位有没有货】—— 有才让抓，没有就让等着
            if sid and self.slots.get(sid, {}).get("occupied"):
                self.pub_cmd(client, device_id, {"cmd": "pick", "task_id": tid, "slot": sid})
                print(f"[平台] 库位 {sid} 有货 → 通知 {device_id} 开始抓取")
            else:
                self.waiting_car[device_id] = {"task_id": tid, "site": task.get("site"),
                                               "slot": sid}
                self.pub_cmd(client, device_id, {"cmd": "wait", "task_id": tid,
                                                 "reason": "货物尚未入库"})
                print(f"[平台] 库位 {sid} 暂无货 → 通知 {device_id} 原地等待（等无人机投放）")
            self._save()
            return

        if event == "pick_done":
            # 取货完成 → 库位置空（★ 影响无人机下次投放）→ 通知小车去送达
            tid = data.get("task_id")
            task = self.tasks.get(tid)
            if task:
                self.slot_clear(task["slot"])
                self.set_task_status(tid, "DELIVERING")
                # ★★★ 装车 = 目的地锁定
                #   ① 把任务里锁定的 dst 回写包裹（把中途改过的覆盖回来）
                #   ② 包裹状态置 LOADED、记下 car_id
                _lock_dst = task.get("dst") or ""
                _pid = task.get("parcel_id")
                try:
                    if _pid:
                        f = {"status": "LOADED", "car_id": device_id}
                        if _lock_dst:
                            f["dst"] = _lock_dst
                        _db.update_parcel(_pid, **f)
                except Exception:
                    pass
                try:
                    _db.update_task(tid, picked_at=int(time.time()),
                                    dst=_lock_dst)      # ★ 取货完成时间 + 锁定目的地
                except Exception:
                    pass
                if _lock_dst:
                    print(f"[平台] 🔒 任务 {tid} 目的地已锁定：{_lock_dst}（装车后不可改）")
                print(f"[平台] 小车 {device_id} 已取走 {task['slot']}（{task['count']} 件）"
                      f"→ 库位置空，任务 {tid} 进入配送")
                self.pub_cmd(client, device_id, {"cmd": "deliver", "task_id": tid,
                                                 "count": task["count"],
                                                 "dst": task.get("dst") or ""})
                self.broadcast(client, {"type": "slot_update", "slot": task["slot"],
                                        "occupied": False})
            self._save()
            self.dump_status()
            return

        if event == "delivered":
            tid = data.get("task_id")
            self.set_task_status(tid, "DONE")
            try:
                _db.update_task(tid, delivered_at=int(time.time()))      # ★ 送达时间
            except Exception:
                pass
            self.devices[device_id]["busy"] = False     # ★ 车空闲了
            print(f"[平台] 小车 {device_id} 已送达，任务 {tid} 完成 ✓")
            self._save()
            self.dispatch(client)                       # ★ 立刻派下一个任务
            self.dump_status()
            return

        # ==================== ★★★ 包裹链路（2026-10-08）====================
        # 目的：物流公司的轨迹到【校门口/驿站】就断了 → 这几条把"校园段"补上
        if event == "parcel_in":
            # ① 中转点工作人员把包裹挂到无人机上 → 建包裹档案
            pid = self.new_parcel(
                tracking_no=str(data.get("tracking_no") or ""),
                order_no=str(data.get("order_no") or ""),
                owner=str(data.get("owner") or data.get("name") or ""),
                phone=str(data.get("phone") or ""),
                dst=str(data.get("dst") or data.get("dest") or ""),
                mode=str(data.get("mode") or "SHELF").upper(),
                drone_id=device_id)
            self.set_parcel(pid, status="ON_DRONE", event="parcel_in",
                            detail="中转点揽收挂载 → 无人机 %s" % device_id,
                            actor=device_id)
            self.pub_cmd(client, device_id, {"cmd": "parcel_ack", "parcel_id": pid,
                                             "pick_code": "", "dst": data.get("dst")})
            self.dump_status()
            return

        if event == "drone_arrive":
            # ② 无人机到达【校门口站点】
            pid = data.get("parcel_id") or ""
            if not pid and data.get("tracking_no"):
                p = _db.find_parcel(tracking_no=str(data["tracking_no"]))
                pid = p["parcel_id"] if p else ""
            if not pid:
                print("[平台] ⚠️ drone_arrive 没带 parcel_id/运单号，忽略")
                return
            self.set_parcel(pid, status="AT_GATE", event="drone_arrive",
                            detail="无人机 %s 送达 %s" % (device_id, data.get("site") or "校门口站点"),
                            actor=device_id)
            print("[平台] \u2605 %s 已到校门口站点 → 等小车取件" % pid)
            self.dump_status()
            return

        if event == "car_load":
            # ③ 小车装载。两种匹配方式：
            #    ★ 有运单号 → 精确匹配（机械臂扫码）
            #    ★★ 没运单号 → 自动取"最近一件已到站点、还没分给车"的包裹（当前阶段用这个）
            pid = data.get("parcel_id") or ""
            p = None
            if data.get("tracking_no"):
                p = _db.find_parcel(tracking_no=str(data["tracking_no"]))
                if p:
                    pid = p["parcel_id"]
                    print("[平台] \u2605 扫码识别：运单号 %s → 包裹 %s (%s)"
                          % (data["tracking_no"], pid, p.get("owner")))
            # ★★★ 第三种匹配：AR 码序号（小车摄像头读出来的"货物身份证"）
            #   ★ 语义：只要摄像头读到了序号，就以它为准；
            #     读到了却找不到对应包裹 → 【不许再自动瞎猜】✗ 直接告警停下，
            #     否则会把别人的货装到车上（真实踩过：9 号货把张三的 P0004 装走了）✗
            _had_marker = data.get("marker_id") not in (None, "", "None")
            _mid = None
            if _had_marker:
                try:
                    _mid = int(data.get("marker_id"))
                except Exception:
                    _mid = None
            if not pid and _mid is not None:
                p = _db.find_parcel_by_marker(_mid)
                if p:
                    pid = p["parcel_id"]
                    print("[平台] ★ AR码识别：序号 %s → 包裹 %s（%s）"
                          % (_mid, pid, p.get("owner")))
                else:
                    print("[平台] ⚠️⚠️ AR码序号 %s 找不到对应包裹 —— "
                          "拒绝装载（不瞎猜是哪件）" % _mid)
                    self.devices[device_id]["busy"] = True
                    return
            # ★ 自动匹配（按"最早到站点"猜）只在【摄像头没读到码】时才用
            if not pid and not _had_marker:
                for cand in _db.list_parcels(limit=100, status="AT_GATE"):
                    if not cand.get("car_id"):
                        p, pid = cand, cand["parcel_id"]
                        print("[平台] \u2605 自动匹配：%s 装载了待取包裹 %s（%s）"
                              % (device_id, pid, cand.get("owner")))
                        break
            if not pid:
                print("[平台] ⚠️ car_load 没找到可装载的包裹（无 AT_GATE 待取件）")
                self.devices[device_id]["busy"] = True
                return
            # ★ 校验：摄像头读到的序号 和 任务里锁定的序号 对得上吗
            _tid = data.get("task_id")
            _t = self.tasks.get(_tid) or {}
            _real = data.get("marker_id")
            if _real not in (None, "", "None") and _t.get("marker_id") is not None:
                try:
                    if int(_real) != int(_t["marker_id"]):
                        print("[平台] ⚠️⚠️ 疑似取错件！任务 %s 要的是 %s 号，"
                              "实际读到 %s 号" % (_tid, _t["marker_id"], _real))
                except Exception:
                    pass
            self.set_parcel(pid, status="LOADED", event="car_load",
                            detail="小车 %s 装载（运单号 %s）" % (device_id, data.get("tracking_no") or "-"),
                            actor=device_id, car_id=device_id, loaded_at=int(time.time()))
            self.car_parcel[device_id] = pid          # ★ 记住这台车装着这件货
            self.devices[device_id]["busy"] = True
            return

        if event == "car_depart":
            pid = data.get("parcel_id") or self.car_parcel.get(device_id, "")
            if pid:
                self.set_parcel(pid, status="DELIVERING", event="car_depart",
                                detail="小车 %s 出发配送" % device_id, actor=device_id)
            return

        if event == "dst_arrive":
            pid = data.get("parcel_id") or self.car_parcel.get(device_id, "")
            if pid:
                self.set_parcel(pid, event="dst_arrive",
                                detail="到达 %s" % (data.get("dst") or "目的地"), actor=device_id)
            return

        if event == "shelf_place":
            # ④ 机械臂把包裹放进【低层货柜】
            pid = data.get("parcel_id") or self.car_parcel.get(device_id, "")
            shelf = str(data.get("shelf_no") or "")
            if pid:
                p0 = _db.get_parcel(pid) or {}
                shelf = shelf or p0.get("shelf_no") or ""
                code = p0.get("pick_code") or data.get("pick_code") or ""
                self.set_parcel(pid, status="PLACED", event="shelf_place",
                                detail="放入货柜 %s（取件码 %s）" % (shelf or "货柜", code),
                                actor=device_id, placed_at=int(time.time()),
                                **({"shelf_no": shelf} if shelf else {}))
                print("[平台] \u2605 %s 已放入 %s → 通知 %s 凭取件码 %s 自取"
                      % (pid, shelf or "货柜", p0.get("owner") or "收件人", code))
            self.car_parcel.pop(device_id, None)        # ★ 这趟结束，解除绑定
            self.devices[device_id]["busy"] = False     # ★ 车这一趟完成
            self.dump_status()
            return

        if event == "picked_up":
            # ⑤ 人扫码取件 → 这件包裹闭环完成
            pid = data.get("parcel_id") or ""
            if not pid and data.get("pick_code"):
                for p in _db.list_parcels(limit=500):
                    if p.get("pick_code") == str(data["pick_code"]) and p.get("status") != "PICKED":
                        pid = p["parcel_id"]; break
            if not pid:
                print("[平台] ⚠️ picked_up 未匹配到包裹")
                return
            self.set_parcel(pid, status="PICKED", event="picked_up",
                            detail="%s 扫码取件" % (data.get("by") or data.get("owner") or "收件人"),
                            actor="user", picked_at=int(time.time()))
            print("[平台] \u2605 %s 已被取走 → 闭环完成 ✓" % pid)
            self.dump_status()
            return

        if event == "battery":
            # ★ 设备电量上报（小车协议 0x07 电池功能码）
            try:
                _db.update_device_battery(device_id,
                                          battery=float(data.get("percent")) if data.get("percent") is not None else None,
                                          voltage=float(data.get("voltage")) if data.get("voltage") is not None else None,
                                          name=str(data.get("name") or ""))
            except Exception:
                pass
            return

        if event == "position":
            # ★ 位置上报（高频，不打日志）：存进数据库 → 网页地图画车用
            try:
                _x = float(data.get("x", 0.0)); _y = float(data.get("y", 0.0))
                _db.update_device_pos(device_id, _x, _y,
                                      float(data.get("yaw", 0.0)),
                                      role=info.get("role", ""),
                                      status="busy" if info.get("busy") else "idle")
                # ★★ 记一个轨迹点（地图上画"走过的路径"）
                _db.add_track(device_id, x=_x, y=_y)
            except Exception:
                pass
            return

        if event == "gps":
            # ★★★ 无人机 GPS 上报（经纬度）→ 存库 + 记轨迹
            #     无人机是【人操作的】：人在网页点"起飞/已取货/已放货"，
            #     飞行中 NX 持续上报 GPS（或人手动点地图标点）
            try:
                lat = float(data.get("lat")); lon = float(data.get("lon"))
                alt = float(data.get("alt") or 0.0)
                _db.update_device_gps(device_id, lat, lon, alt,
                                      role=info.get("role", "drone"),
                                      status="busy" if info.get("busy") else "idle")
                _db.add_track(device_id, lat=lat, lon=lon, alt=alt)
            except Exception:
                pass
            return

        if event == "exception":
            _why = str(data.get("reason") or "")
            _tid = data.get("task_id")
            print(f"[平台] ⚠️ 设备 {device_id} 上报异常：{_why}（任务 {_tid}）")

            # ★★★ 货物校验失败（疑似中途被调包）→ 醒目告警 + 写进事件表
            #   场景：小车到货格取货，摄像头读到的 AR 码 ≠ 平台记录的那件
            #   处置：不继续自动跑，车停住等人工确认（安全第一）
            if any(k in _why for k in ("校验", "调包", "不是任务", "期望")):
                print(f"[平台] 🚨🚨 【货物校验失败】{device_id} 在库位 {data.get('slot')} "
                      f"读到的不是任务要的那件货！→ 已停住等人工处置")
                try:
                    _db.log_event(device_id, "cargo_mismatch",
                                  {"task_id": _tid, "slot": data.get("slot"),
                                   "detail": _why}, actor=device_id)
                except Exception:
                    pass
                try:
                    self.devices[device_id]["busy"] = False   # 别再自动派新任务
                except Exception:
                    pass
            return

        print(f"[平台] ? 未知事件 {event}，来自 {device_id}：{data}")


# ==================== MQTT 接入 ====================
def _acquire_singleton_lock():
    """★★★ 单例锁：防止同一个平台跑出多个进程

    为什么需要：反复 `nohup python3 hub.py &` 而旧的没杀掉时，
    会有多个平台进程【同时订阅 cjy/# 抢消息】→ 状态错乱
    （症状：货架一会儿空一会儿有货、任务莫名多出来一堆）
    """
    import os as _os
    import errno
    lock = "/tmp/cjy-platform.lock"
    try:
        f = open(lock, "w")
        f.write(str(_os.getpid()))
        f.flush()
        try:
            import fcntl
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except ImportError:
            pass                          # 非 POSIX 就退化成"只写 PID"
        except (IOError, OSError):
            print("⚠️ 已经有一个平台在跑了（锁 %s）。")
            print("   先杀掉旧的：ps -eo pid,cmd | grep hub | grep -v grep")
            sys.exit(1)
        return f
    except Exception:
        return None


def main():
    _lock = _acquire_singleton_lock()     # ★★ 单例锁
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
            # ★★ 初始化校内【配送点货柜】（只做一次；MQTT 重连不重复打印）
            if not getattr(plat, "_dst_ready", False):
                plat._dst_ready = True
                try:
                    _db.ensure_dst_slots()
                    n = len(_db.list_dst_slots())
                    print(f"[平台] ★ 配送点货柜已就绪（{n} 格，收件人来取货的地方）")
                except Exception as e:
                    print(f"[平台] ⚠️ 初始化配送点货柜失败：{e}")
            # ★ 库位巡检：连上就启动（每 5 秒扫一次，有货无任务 → 主动派车）
            if not getattr(plat, "_patrol_started", False):
                plat._patrol_started = True
                plat.start_patrol(c, interval=5.0)
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
