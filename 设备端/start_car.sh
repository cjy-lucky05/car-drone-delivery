#!/bin/bash
# ============================================================================
#  车-机协同末端配送 · 小车端【一键启动】
# ============================================================================
#  用法：
#      bash start_car.sh              # 全自动启动（不开 RViz，正常跑就用这个）
#      bash start_car.sh --rviz       # 带 RViz（调试 / 看地图时用）
#      bash start_car.sh --dry        # 干跑模式（不真导航，只验证链路）
#
#  启动顺序：roscore → 导航 → 单位转换 → 机械臂 → 初始位姿 → 桥接节点
#  启动完成后等平台指令，无需人工操作 ✓
# ============================================================================
set -u

# ---------------------------- 配置（改这里）----------------------------
CAR_DIR="$HOME/chen_car_drone/小车"
LOG_DIR="$HOME/chen_car_drone/logs"
CATKIN_DIR="$HOME/catkin_ws"

# ★ 车的固定起始位姿（每次都从同一位置出发 → 自动给 AMCL，不用手动点 RViz）
#   格式: x y yaw角度（米 / 米 / 度）
START_X="-0.08"
START_Y="0.00"
START_YAW="0"          # 0 = 车头朝地图 +x 方向

# 自动给初始位姿：1=开  0=关（想手动在 RViz 点就改 0）
AUTO_INIT_POSE="1"
# ----------------------------------------------------------------------

OPEN_RVIZ="false"
DRY_FLAG=""
for a in "$@"; do
    case "$a" in
        --rviz) OPEN_RVIZ="true" ;;
        --dry)  DRY_FLAG="--dry" ;;
    esac
done

mkdir -p "$LOG_DIR"
source /opt/ros/noetic/setup.bash 2>/dev/null
source "$CATKIN_DIR/devel/setup.bash" 2>/dev/null

echo "=============================================================="
echo "  小车端一键启动    $(date '+%Y-%m-%d %H:%M:%S')"
echo "  RViz: $OPEN_RVIZ   干跑: ${DRY_FLAG:-否}   自动位姿: $AUTO_INIT_POSE"
echo "=============================================================="

# ---------- 0. 先清理残留 ----------
echo "[0/6] 清理残留进程…"
bash "$CAR_DIR/stop_car.sh" >/dev/null 2>&1
sleep 1

# ---------- 1. roscore ----------
echo "[1/6] 启动 roscore…"
nohup roscore > "$LOG_DIR/roscore.log" 2>&1 &
echo $! > "$LOG_DIR/roscore.pid"
for i in $(seq 1 20); do
    if rosparam list >/dev/null 2>&1; then echo "      OK roscore 就绪"; break; fi
    sleep 1
    [ "$i" = "20" ] && { echo "      !! roscore 起不来，看 $LOG_DIR/roscore.log"; exit 1; }
done

# ---------- 2. 导航 ----------
echo "[2/6] 启动导航（底盘 + 雷达 + amcl + move_base）…"
nohup roslaunch wd_robot_navigation robot_navigation.launch open_rviz:=$OPEN_RVIZ \
    > "$LOG_DIR/navigation.log" 2>&1 &
echo $! > "$LOG_DIR/navigation.pid"
for i in $(seq 1 60); do
    if rostopic list 2>/dev/null | grep -q "/move_base/status"; then echo "      OK 导航就绪"; break; fi
    sleep 1
    [ "$i" = "60" ] && echo "      !! 导航没就绪，看 $LOG_DIR/navigation.log"
done

# ---------- 3. 关节状态单位转换（度 → 弧度）----------
#   ★ 必需！底盘驱动的关节状态是【度】，MoveIt 要【弧度】
#     没有它 → MoveIt 报 "invalid start state" → 机械臂规划失败
echo "[3/6] 启动关节状态单位转换节点（度 → 弧度）…"
cd "$CAR_DIR" || exit 1
nohup python3 -u "$CAR_DIR/joint_deg2rad.py" > "$LOG_DIR/deg2rad.log" 2>&1 &
echo $! > "$LOG_DIR/deg2rad.pid"
sleep 2
# 检测依据：驱动改名后的 /joint_states_deg 出现 = 转换链路通了
if rostopic list 2>/dev/null | grep -q "/joint_states_deg"; then
    echo "      OK 转换节点就绪（/joint_states 已转为弧度）"
else
    echo "      !! 转换链路没通，看 $LOG_DIR/deg2rad.log 和 $LOG_DIR/navigation.log"
fi

# ---------- 4. 机械臂 ----------
echo "[4/6] 启动机械臂（MoveIt + 执行节点 + moveit_pose）…"
nohup roslaunch "$CAR_DIR/robot_arm.launch" > "$LOG_DIR/arm.log" 2>&1 &
echo $! > "$LOG_DIR/arm.pid"
for i in $(seq 1 60); do
    if rostopic list 2>/dev/null | grep -q "/move_group/result"; then echo "      OK 机械臂就绪"; break; fi
    sleep 1
    [ "$i" = "60" ] && echo "      !! 机械臂没就绪，看 $LOG_DIR/arm.log"
done

# ---------- 5. 自动给 AMCL 初始位姿 ----------
if [ "$AUTO_INIT_POSE" = "1" ]; then
    echo "[5/6] 自动下发 AMCL 初始位姿 (x=$START_X y=$START_Y yaw=${START_YAW}度)…"
    YAW_RAD=$(python3 -c "import math;print(${START_YAW}*math.pi/180)")
    QZ=$(python3 -c "import math;print(math.sin($YAW_RAD/2))")
    QW=$(python3 -c "import math;print(math.cos($YAW_RAD/2))")
    rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
      "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $START_X, y: $START_Y, z: 0.0}, orientation: {z: $QZ, w: $QW}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, 0,0,0,0.25,0,0, 0,0,0,0,0.25,0, 0,0,0,0,0,0.07]}}" \
      > /dev/null 2>&1
    echo "      OK 初始位姿已下发"
else
    echo "[5/6] 跳过初始位姿（AUTO_INIT_POSE=0，请自己在 RViz 点 2D Pose Estimate）"
fi

# ---------- 6. 桥接节点 ----------
echo "[6/6] 启动 MQTT 桥接节点…"
cd "$CAR_DIR" || exit 1
nohup python3 -u mqtt_bridge_node.py $DRY_FLAG > "$LOG_DIR/bridge.log" 2>&1 &
echo $! > "$LOG_DIR/bridge.pid"
sleep 3

echo
echo "=============================================================="
echo "  ✅ 全部启动完成 —— 现在等平台/前端下发指令即可"
echo "=============================================================="
echo "  实时看日志："
echo "     tail -f $LOG_DIR/bridge.log        ← 桥接节点（最常看）"
echo "     tail -f $LOG_DIR/navigation.log    ← 导航"
echo "     tail -f $LOG_DIR/arm.log           ← 机械臂"
echo "     tail -f $LOG_DIR/deg2rad.log       ← 单位转换"
echo "  停止全部：  bash $CAR_DIR/stop_car.sh"
echo "=============================================================="
echo
echo "--- 桥接节点启动情况（最后 6 行）---"
tail -6 "$LOG_DIR/bridge.log" 2>/dev/null || echo "（日志还没内容，稍等几秒）"
echo "=============================================================="
