#!/bin/bash
# ============================================================
# start_all.sh —— 一键检查并启动：mosquitto + 平台 hub.py
#
# 用法（服务器上）：bash /root/chenjiayu/scripts/start_all.sh
#
# 做什么：逐个检查、没在跑就启动、最后打印状态。
# 幂等：已经在跑就不会重复启动（不会起出多个进程互相踢）。
# ============================================================

PROJ=/root/chenjiayu
PORT=1884

echo "╔══════════════════════════════════════════════╗"
echo "║  车-机协同平台 启动检查                      ║"
echo "╚══════════════════════════════════════════════╝"

# ---------- 1. mosquitto ----------
echo
echo "===== 1. mosquitto (broker) ====="
if ss -tlnp 2>/dev/null | grep -q ":$PORT "; then
    echo "  ✓ 已在运行"
    ps -eo pid,lstart,cmd | grep [m]osquitto
else
    echo "  ✗ 没在跑 → 启动中…"
    pkill mosquitto 2>/dev/null
    sleep 2
    mosquitto -c $PROJ/mqtt/config/mosquitto.conf -d
    sleep 1
    if ss -tlnp 2>/dev/null | grep -q ":$PORT "; then
        echo "  ✓ 启动成功（端口 $PORT 已监听）"
    else
        echo "  ✗ 启动失败！最近日志："
        tail -10 $PROJ/logs/mosquitto.log
    fi
fi

# ---------- 2. 平台 hub.py ----------
echo
echo "===== 2. 平台 hub.py ====="
CNT=$(pgrep -f hub.py | wc -l)
if [ "$CNT" -eq 1 ]; then
    echo "  ✓ 已在运行"
    pgrep -af hub.py
elif [ "$CNT" -gt 1 ]; then
    echo "  ⚠️ 有 $CNT 个进程在跑（会互相踢）→ 清理后重启"
    pkill -f hub.py
    sleep 2
    cd $PROJ && nohup python3 hub.py > logs/platform.log 2>&1 &
    sleep 2
    pgrep -af hub.py
else
    echo "  ✗ 没在跑 → 启动中…"
    cd $PROJ && nohup python3 hub.py > logs/platform.log 2>&1 &
    sleep 2
    if pgrep -f hub.py > /dev/null; then
        echo "  ✓ 启动成功"
        pgrep -af hub.py
    else
        echo "  ✗ 启动失败！最近日志："
        tail -10 $PROJ/logs/platform.log
    fi
fi

# ---------- 3. 状态 ----------
echo
echo "===== 3. 平台当前状态 ====="
tail -8 $PROJ/logs/platform.log

echo
echo "──────────────────────────────────────────────"
echo "  看实时日志： tail -f $PROJ/logs/platform.log"
echo "  模拟设备端： 在你 Windows 上跑 设备端\\device_client.py"
echo "──────────────────────────────────────────────"
