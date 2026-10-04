# -*- coding: utf-8 -*-
"""
项目配置【模板】
================
用法：把本文件复制成 config_local.py，填入真实值。
      config_local.py 已在 .gitignore 里，不会被提交。

  Windows:  copy config.example.py config_local.py
  Linux  :  cp config.example.py config_local.py

也可以不建这个文件，改用环境变量（名字相同）：
  export MQTT_BROKER=...   export MQTT_PASS_CAR=...
"""

# ---------- MQTT 服务器 ----------
MQTT_BROKER = "101.37.242.91"      # 平台在服务器本机跑时用 127.0.0.1
MQTT_PORT   = 1884
MQTT_NS     = "cjy"                # 主题命名空间（和同 broker 上别人的消息隔离）

# ---------- 各角色账号 ----------
MQTT_USER_CAR   = "cjy-car"
MQTT_PASS_CAR   = ""               # ← 填小车端密码

MQTT_USER_DRONE = "cjy-drone"
MQTT_PASS_DRONE = ""               # ← 填无人机端密码

MQTT_USER_HUB   = "cjy-car"        # 平台暂借小车账号（原 cjy-platform 有认证问题）
MQTT_PASS_HUB   = ""               # ← 填平台用的密码

# ---------- 设备编号 ----------
DEVICE_ID_CAR   = "car-01"
DEVICE_ID_DRONE = "nx-01"
DEVICE_ID_HUB   = "hub"
