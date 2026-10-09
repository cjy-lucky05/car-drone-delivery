# -*- coding: utf-8 -*-
"""
从网页下发指令给设备（走 MQTT）
================================
★ 前端【不直接写数据库】，人的操作以 MQTT 消息的形式发给 hub.py，
  由 hub.py 统一决策 + 写库 —— 这样仍是"单一真相源"，零并发写问题 ✓

账号密码从 config_local.py 读（不进仓库）
"""
import json
import os
import sys
from typing import Any, Dict, Optional

try:
    import config_local as _CFG
except ImportError:
    _CFG = None


def _cfg(key: str, default: str = "") -> str:
    v = getattr(_CFG, key, None) if _CFG else None
    return v if v not in (None, "") else os.getenv(key, default)


BROKER = _cfg("MQTT_BROKER", "127.0.0.1")
PORT = int(_cfg("MQTT_PORT", "1884"))
USERNAME = _cfg("MQTT_USER_HUB", _cfg("MQTT_USER_CAR", ""))
PASSWORD = _cfg("MQTT_PASS_HUB", _cfg("MQTT_PASS_CAR", ""))
NS = _cfg("MQTT_NS", "cjy")


def publish(topic: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    """发一条 MQTT 消息。返回 {"ok": bool, "detail": str}"""
    if not USERNAME or not PASSWORD:
        return {"ok": False, "detail": "未读到 MQTT 账号密码（请放好 config_local.py）"}
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        return {"ok": False, "detail": "没装 paho-mqtt（pip3 install --target=... paho-mqtt）"}
    try:
        c = mqtt.Client()
        c.username_pw_set(USERNAME, PASSWORD)
        c.connect(BROKER, PORT, 5)
        c.loop_start()
        info = c.publish(topic, json.dumps(payload, ensure_ascii=False), qos=0)
        info.wait_for_publish(timeout=3)
        c.loop_stop()
        c.disconnect()
        return {"ok": True, "detail": "已发送到 " + topic}
    except Exception as e:
        return {"ok": False, "detail": "发送失败: %s" % e}


def send_to_car(car_id: str, cmd: str, **extra) -> Dict[str, Any]:
    payload = {"cmd": cmd, "from": "web"}
    payload.update(extra)
    return publish("%s/%s/cmd" % (NS, car_id), payload)


def send_to_drone(drone_id: str, cmd: str, **extra) -> Dict[str, Any]:
    payload = {"cmd": cmd, "from": "web"}
    payload.update(extra)
    return publish("%s/%s/cmd" % (NS, drone_id), payload)


def report_as_device(device_id: str, role: str, event: str, **extra) -> Dict[str, Any]:
    """★★★ 以【设备身份上报】一条事件（发到 cjy/<id>/report）

    用途：无人机是【人操作的】—— 人在网页上点"起飞/已取货/已放货"，
    平台就替无人机发一条上报，走的是和真机上报【完全一样】的通道。
    这样 hub.py 的既有逻辑（登记库位 / 生成任务 / 派车）全部复用，零改动 ✓
    """
    import time as _t
    payload = {"device": device_id, "role": role, "event": event,
               "ts": int(_t.time()), "from": "web"}
    payload.update(extra)
    return publish("%s/%s/report" % (NS, device_id), payload)
