#!/bin/bash
# ============================================================================
#  车-机协同末端配送 · 小车端【一键停止】
#  用法：bash stop_car.sh
#  说明：只杀本项目的进程（用关键词精确匹配，不用通配符乱杀）
# ============================================================================
LOG_DIR="$HOME/chen_car_drone/logs"

echo "停止小车端全部节点…"

# ---------- 1. 按 PID 文件停（精确）----------
for name in bridge arm deg2rad navigation roscore; do
    pidfile="$LOG_DIR/$name.pid"
    if [ -f "$pidfile" ]; then
        pid=$(cat "$pidfile")
        if [ -n "$pid" ]; then
            kill "$pid" 2>/dev/null && echo "  - 已停 $name (pid $pid)"
        fi
        rm -f "$pidfile"
    fi
done
sleep 2

# ---------- 2. 按关键词兜底（roslaunch 的子节点可能残留）----------
for kw in \
    "mqtt_bridge_node.py" \
    "joint_deg2rad.py" \
    "moveit_pose.py" \
    "SimulationToMachine" \
    "wd_arm_moveit_demo" \
    "move_group" \
    "wd_vision" \
    "robot_navigation.launch" \
    "move_base" \
    "amcl" \
    "map_server" \
    "ydlidar" \
    "wd_base_control" \
    "joint_state_publisher" \
    "robot_state_publisher" \
    "rviz" \
    "roscore" \
    "rosmaster" \
    "rosout"
do
    pkill -f "$kw" 2>/dev/null
done
sleep 2

# ---------- 3. 确认 ----------
left=$(ps -ef | grep -E "mqtt_bridge_node|move_base|rosmaster|move_group" | grep -v grep | wc -l)
if [ "$left" = "0" ]; then
    echo "  ✅ 已全部停止"
else
    echo "  ⚠️  还剩 $left 个相关进程："
    ps -ef | grep -E "mqtt_bridge_node|move_base|rosmaster|move_group" | grep -v grep
fi
