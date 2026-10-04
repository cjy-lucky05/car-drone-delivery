#!/bin/bash
# ============================================================
# restart_mqtt.sh —— 安全重启 mosquitto
#
# 为什么需要它：
#   mosquitto 只在【启动时】读一次密码文件。而 `pkill mosquitto` 之后
#   立刻启动，旧进程可能还没释放 1884 端口 → 新进程静默启动失败
#   → 结果是旧进程继续跑、用旧密码表，新加的用户/新改的密码都不生效。
#   本脚本用「pkill → sleep 2 → 确认停干净 → 再启动 → 确认起来」规避它。
#
# 用法（服务器上）：
#   bash /root/chenjiayu/scripts/restart_mqtt.sh
# ============================================================

CONF=/root/chenjiayu/mqtt/config/mosquitto.conf
LOG=/root/chenjiayu/logs/mosquitto.log
PORT=1884

echo "===== 1. 停止旧的 ====="
pkill mosquitto
sleep 2
if pgrep mosquitto > /dev/null; then
    echo "  还有残留进程，强制结束"
    pkill -9 mosquitto
    sleep 1
fi
if pgrep mosquitto > /dev/null; then
    echo "  ✗ 停不掉，请手动检查：ps -eo pid,cmd | grep [m]osquitto"
    exit 1
fi
echo "  ✓ 已停干净"

echo "===== 2. 启动新的 ====="
mosquitto -c "$CONF" -d
sleep 1

echo "===== 3. 确认 ====="
if ! pgrep mosquitto > /dev/null; then
    echo "  ✗ 启动失败！最近日志："
    tail -10 "$LOG"
    exit 1
fi
ps -eo pid,lstart,cmd | grep [m]osquitto
ss -tlnp | grep ":$PORT" || echo "  ⚠️ $PORT 端口没在监听，检查日志"
echo "  ✓ 启动成功"

echo "===== 4. 最近日志 ====="
tail -5 "$LOG"
