# -*- coding: utf-8 -*-
"""
本地配置模板
=============
用法：把这个文件复制成 config_local.py，填上真实密码。
★ config_local.py 已被 .gitignore 排除，永不提交。

（本项目四台机器通用：服务器 / 小车 / 无人机 / 电脑）
"""
# MQTT 服务器
MQTT_BROKER = "101.37.242.91"       # 服务器上跑 FastAPI 时改成 127.0.0.1
MQTT_PORT   = 1884
MQTT_NS     = "cjy"

# 账号（按需填）
MQTT_USER_CAR    = "cjy-car"
MQTT_PASS_CAR    = "在这里填小车密码"

MQTT_USER_DRONE  = "cjy-drone"
MQTT_PASS_DRONE  = "在这里填无人机密码"

# 平台/网页用的账号（没有独立的就沿用 cjy-car）
MQTT_USER_HUB    = "cjy-car"
MQTT_PASS_HUB    = "在这里填平台密码"
