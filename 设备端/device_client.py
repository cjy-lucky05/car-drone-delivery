# -*- coding: utf-8 -*-
"""
设备端 MQTT 客户端（小车 / 无人机端 通用）
==========================================
连接 MQTT broker（地址见 config_local.py），上报事件、订阅指令。

【常用命令】
  # ★★ 无人机 GPS 位置上报（人飞的时候让平台地图上看到机在哪）
  python3 device_client.py --role drone --gps 34.3416 108.9398            # 手动给经纬度（示例）
  python3 device_client.py --role drone --gps-auto                        # 自动读 gpsd（每 5 秒一次）
  python3 device_client.py --role drone --gps-auto --gps-iface wlan0      # 指定网卡/串口给 gpsd

  # 自检（连上、发一条就退出）
  python device_client.py --role car --test

  # 常驻运行（收平台的指令）
  python device_client.py --role car

  # 上报一个事件（发完监听 5 秒，能看到平台回的指令）
  python device_client.py --role drone --event ask_slot  --site 1
  python device_client.py --role drone --event drop_done --site 1 --extra "{\"slot\":\"1A\",\"count\":2}"
  python device_client.py --role car   --event pick_done --extra "{\"task_id\":\"T0001\",\"slot\":\"1A\",\"count\":2}"
  python device_client.py --role car   --event delivered --extra "{\"task_id\":\"T0001\"}"

依赖：pip install paho-mqtt
"""

import argparse
import json
import os
import sys
import threading
import time

# ★ 自动寻找依赖目录（同一份脚本在 服务器 / NX(U盘) / 小车 都能用）
for _p in ("/mnt/usb/chenjiayu/libs",                     # NX（U 盘）
           os.path.expanduser("~/chen_car_drone/libs"),   # 小车
           os.path.expanduser("~/libs"),                  # 用户目录
           "/root/chenjiayu/libs"):                       # 服务器
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

import paho.mqtt.client as mqtt

# ==================== broker ====================
# ---- 配置读取：优先 config_local.py，其次环境变量（★ 明文密码不进仓库）----
try:
    import config_local as _CFG
except ImportError:
    _CFG = None

def _cfg(key, default=""):
    v = getattr(_CFG, key, None) if _CFG else None
    return v if v not in (None, "") else os.getenv(key, default)

BROKER = _cfg("MQTT_BROKER", "")      # ★ 服务器地址属隐私 → 放 config_local.py（代码里留空）
PORT   = int(_cfg("MQTT_PORT", "1884"))

# ==================== 设备身份表 ====================
# ★ 含明文密码，别外传、别提交到公开仓库
DEVICES = {
    "car": {
        "username": _cfg("MQTT_USER_CAR", "cjy-car"),
        "password": _cfg("MQTT_PASS_CAR", ""),
        "device_id": _cfg("DEVICE_ID_CAR", "car-01"),
        "site": "1",
    },
    "drone": {
        "username": _cfg("MQTT_USER_DRONE", "cjy-drone"),
        "password": _cfg("MQTT_PASS_DRONE", ""),
        "device_id": _cfg("DEVICE_ID_DRONE", "nx-01"),
        "site": "",
    },
}

TOPIC_BROADCAST = "cjy/broadcast"


# ==================== 回调 ====================
def on_connect(client, userdata, flags, reason_code, properties=None, cfg=None):
    did = cfg["device_id"]
    if reason_code == 0:
        print(f"[{did}] 已连接 {BROKER}:{PORT}")
        client.subscribe(f"cjy/{did}/cmd")
        client.subscribe(TOPIC_BROADCAST)
        print(f"[{did}] 已订阅：cjy/{did}/cmd , {TOPIC_BROADCAST}")
    else:
        print(f"[{did}] 连接失败 rc={reason_code}（密码错？端口没放行？）")


def on_message(client, userdata, msg):
    payload = msg.payload.decode("utf-8", errors="replace")
    print(f"  📩 收到 {msg.topic}: {payload}")
    try:
        handle_command(json.loads(payload))
    except Exception:
        pass


def on_disconnect(client, userdata, disconnect_flags=None, reason_code=None, properties=None):
    print(f"  ⚠️ 连接断开 rc={reason_code}")


def handle_command(data: dict):
    cmd = data.get("cmd")
    if cmd == "use_slot":
        print(f"     → 平台让我放到库位 {data.get('slot')}")
    elif cmd == "wait":
        print(f"     → 平台说先等：{data.get('reason')}")
    elif cmd == "goto_pick":
        print(f"     → 取货任务 {data.get('task_id')}：去站点 {data.get('site')} 库位 {data.get('slot')}（{data.get('count')} 件）")
    elif cmd == "deliver":
        print(f"     → 去送达，任务 {data.get('task_id')}（{data.get('count')} 件）")
    elif cmd == "ping":
        print("     → 收到 ping")
    else:
        print(f"     → 指令：{data}")


# ==================== 主流程 ====================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", choices=["car", "drone"], default="car", help="设备身份")
    ap.add_argument("--test", action="store_true", help="自检：连上、发一条、退出")
    ap.add_argument("--event", help="上报的事件名，如 ask_slot / drop_done / arrived / pick_done / delivered")
    ap.add_argument("--site", help="站点号，如 1")
    ap.add_argument("--extra", help='附加字段（JSON 字符串），如 \'{"task_id":"T0001"}\'')
    ap.add_argument("--wait", type=int, default=5, help="发完后监听几秒（默认 5，用于看平台回复）")
    ap.add_argument("--gps", nargs=2, metavar=("LAT", "LON"), help="★ 上报一次 GPS 经纬度")
    ap.add_argument("--gps-auto", action="store_true", help="★ 自动读 GPS（gpsd）并周期性上报")
    ap.add_argument("--gps-interval", type=float, default=5.0, help="GPS 上报间隔秒（默认 5）")
    args = ap.parse_args()

    cfg = DEVICES[args.role]
    # ★★★ 没读到服务器地址 → 明确报错（不然会去连字符串"你的MQTT服务器IP"，只报域名解析失败）
    if (not BROKER) or ("你的" in BROKER) or ("服务器" in BROKER):
        print("❌ 没读到 MQTT 服务器地址（config_local.py 里缺 MQTT_BROKER）")
        print('   请在 config_local.py 里补一行：MQTT_BROKER = "1.2.3.4"')
        print("   或用环境变量：MQTT_BROKER=1.2.3.4 python3 device_client.py --role drone")
        sys.exit(1)
    # ★ 没读到密码就直接报错退出（不静默用空密码，否则只表现为"认证失败"，难排查）
    if not cfg["password"]:
        print(f"⚠️ 角色 {args.role} 未读到密码！请复制 config.example.py 为 config_local.py，"
              f"并填写 MQTT_PASS_{args.role.upper()}")
        sys.exit(1)
    did = cfg["device_id"]

    # ★★ client_id 必须唯一：常驻模式用设备号；
    #    CLI 发事件模式加随机后缀 → 否则会跟常驻那个【互相踢下线】
    #    （症状：反复 "连接断开 rc=Unspecified error"）
    if args.event or args.test:
        _cid = "%s-cli%d" % (did, int(time.time()) % 100000)
    else:
        _cid = did
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=_cid)
    client.username_pw_set(cfg["username"], cfg["password"])
    client.on_connect = lambda c, u, f, rc, p=None: on_connect(c, u, f, rc, p, cfg=cfg)
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    try:
        client.connect(BROKER, PORT, keepalive=60)
    except Exception as e:
        print(f"[{did}] 连不上 {BROKER}:{PORT} → {e}")
        print("   检查：① 密码 ② 1884 端口有没有放行 ③ 网络")
        sys.exit(1)

    # ---------- ★★ GPS 上报（无人机位置）----------
    def _read_gpsd():
        """从 gpsd 读一次经纬度；读不到返回 None。
        需要 NX 上装了 gpsd（sudo apt install gpsd gpsd-clients python3-gps）
        """
        try:
            import socket as _s
            c = _s.socket(_s.AF_INET, _s.SOCK_STREAM)
            c.settimeout(2.0)
            c.connect(("127.0.0.1", 2947))
            c.sendall(b'?WATCH={"enable":true,"json":true}\n')
            buf = b""
            for _ in range(20):
                buf += c.recv(4096)
                for line in buf.split(b"\n"):
                    if not line.strip():
                        continue
                    try:
                        j = json.loads(line.decode("utf-8", "replace"))
                    except Exception:
                        continue
                    if j.get("class") == "TPV" and j.get("lat") is not None:
                        c.close()
                        return (float(j["lat"]), float(j["lon"]),
                                float(j.get("alt") or 0.0))
            c.close()
        except Exception:
            return None
        return None

    if args.gps or args.gps_auto:
        client.loop_start()
        time.sleep(1)
        once = bool(args.gps)
        while True:
            if args.gps:
                lat, lon = float(args.gps[0]), float(args.gps[1])
                alt = 0.0
            else:
                g = _read_gpsd()
                if not g:
                    print(f"[{did}] ⚠️ 读不到 GPS（gpsd 没起？没接 GPS 模块？）—— 试 \n"
                          f"    sudo systemctl start gpsd   或先用 --gps LAT LON 手动喂")
                    if once:
                        break
                    time.sleep(args.gps_interval)
                    continue
                lat, lon, alt = g
            msg = {"device": did, "role": args.role, "event": "gps",
                   "ts": int(time.time()), "lat": round(lat, 7),
                   "lon": round(lon, 7), "alt": round(alt, 2)}
            client.publish(f"cjy/{did}/report", json.dumps(msg), qos=0)
            print(f"[{did}] 📡 GPS → {msg['lat']}, {msg['lon']}（高 {msg['alt']}m）")
            if once:
                time.sleep(1)
                break
            time.sleep(args.gps_interval)
        client.loop_stop()
        client.disconnect()
        return

    # ---------- 常驻模式 ----------
    if not args.test and not args.event:
        print(f"[{did}] 运行中，Ctrl+C 退出（★ 每 10 秒上报心跳，平台才会显示在线）")

        # ★★ 心跳线程：平台按"最后上报时间"判在线（>60 秒没上报就判离线）
        #    所以常驻模式必须定期上报，否则界面永远不显示本设备在线 ✗
        def _heartbeat():
            while True:
                time.sleep(10)
                try:
                    client.publish(
                        f"cjy/{did}/report",
                        # ★★ 用 "heartbeat" 而不是 "idle"：
                        #    idle 会被平台当成"我空闲了 → 派任务"的信号，
                        #    也会污染"事件记录"。心跳只是保活，单独一个名字。
                        json.dumps({"device": did, "role": args.role,
                                    "event": "heartbeat", "ts": int(time.time()),
                                    "note": "heartbeat"}), qos=0)
                except Exception:
                    pass
        threading.Thread(target=_heartbeat, daemon=True).start()

        try:
            client.loop_forever()
        except KeyboardInterrupt:
            print(f"\n[{did}] 已退出")
            client.disconnect()
        return

    # ---------- 发一条就退出 ----------
    client.loop_start()
    time.sleep(1)

    if args.test:
        msg = {"device": did, "role": args.role, "event": "selftest", "ts": int(time.time())}
    else:
        msg = {"device": did, "role": args.role, "event": args.event, "ts": int(time.time())}
        if args.site:
            msg["site"] = args.site
        if args.extra:
            try:
                msg.update(json.loads(args.extra))
            except Exception as e:
                print(f"❌ --extra 不是合法 JSON：{e}")
                sys.exit(1)

    topic = f"cjy/{did}/report"
    print(f"[{did}] → 上报到频道 {topic}：{json.dumps(msg, ensure_ascii=False)}")
    client.publish(topic, json.dumps(msg, ensure_ascii=False))   # ★ 修：变量名是 client 不是 c

    # 监听一会儿，看平台有没有回指令
    print(f"[{did}] 监听 {args.wait} 秒（等平台回复）…")
    time.sleep(args.wait)

    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()
