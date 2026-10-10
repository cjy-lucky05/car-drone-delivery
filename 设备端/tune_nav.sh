#!/bin/bash
# =====================================================================
# 导航参数修正（2026-10-10）
#
# 解决：快到目标点或行进中【前后左右徘徊】、TEB 反复 "Resetting planner"
#
# 改三处（都在 wd_robot_navigation 包里的配置文件中）：
#   xy_goal_tolerance    0.05 → 0.08   位置容差太紧，阿克曼停不进去 ✗
#   yaw_goal_tolerance   0.05 → 0.10   朝向同理（学长A论文只要求位置 5cm，朝向没收那么紧）
#   controller_frequency 2.0  → 10.0   ★ 这是根因：2Hz 太低 → timediff<=0 / not feasible
#
# 用法（小车上）：
#     bash tune_nav.sh
# 改完【必须重启导航】（参数只在启动时读一次）
# 回滚：同目录下 *.bak.tune 就是原文件
# =====================================================================
set -u

PKG=$(rospack find wd_robot_navigation 2>/dev/null)
if [ -z "$PKG" ]; then
    echo "✗ 找不到 wd_robot_navigation 包（先 source ~/catkin_ws/devel/setup.bash）"
    exit 1
fi
echo "包路径: $PKG"
cd "$PKG" || exit 1

# ① 先找出所有相关的配置文件（launch / yaml / yml）
FILES=$(grep -rl -e "goal_tolerance" -e "controller_frequency" . \
        --include=*.launch --include=*.yaml --include=*.yml 2>/dev/null)
if [ -z "$FILES" ]; then
    echo "✗ 没找到相关配置文件"
    echo "  手动找： grep -rn -e goal_tolerance -e controller_frequency $PKG"
    exit 1
fi

echo
echo "=== 改前 ==="
grep -rn -e "goal_tolerance" -e "controller_frequency" . \
     --include=*.launch --include=*.yaml --include=*.yml 2>/dev/null

for f in $FILES; do
    if [ ! -f "$f.bak.tune" ]; then
        cp "$f" "$f.bak.tune" || { echo "  ✗ 备份失败 $f"; continue; }
    fi
    # 兼容 YAML 写法（xxx: 0.1）和 launch 写法（name="xxx" value="0.1"）
    sed -i \
      -e 's/\(xy_goal_tolerance:[[:space:]]*\)[0-9.]*/\10.08/' \
      -e 's/\(yaw_goal_tolerance:[[:space:]]*\)[0-9.]*/\10.10/' \
      -e 's/\(controller_frequency:[[:space:]]*\)[0-9.]*/\110.0/' \
      -e 's/\(name="xy_goal_tolerance"[^>]*value="\)[0-9.]*/\10.08/' \
      -e 's/\(name="yaw_goal_tolerance"[^>]*value="\)[0-9.]*/\10.10/' \
      -e 's/\(name="controller_frequency"[^>]*value="\)[0-9.]*/\110.0/' \
      "$f"
    echo "  ✔ 已改 $f   （备份: $f.bak.tune）"
done

echo
echo "=== 改后（应为 0.08 / 0.10 / 10.0）==="
grep -rn -e "goal_tolerance" -e "controller_frequency" . \
     --include=*.launch --include=*.yaml --include=*.yml 2>/dev/null

echo
echo "★ 下一步：Ctrl+C 停掉导航 → 重起"
echo "  source ~/catkin_ws/devel/setup.bash && roslaunch wd_robot_navigation robot_navigation.launch open_rviz:=true map_file:=\$HOME/chen_car_drone/maps/lab_20261009.yaml"
echo
echo "★ 起完确认生效（三条都该是新值）："
echo "  rosparam get /move_base/TebLocalPlannerROS/xy_goal_tolerance      → 0.08"
echo "  rosparam get /move_base/TebLocalPlannerROS/yaw_goal_tolerance     → 0.1"
echo "  rosparam get /move_base/controller_frequency                      → 10.0"
echo
echo "★★ 观察点：TEB 那两条报错（timediff<=0 / trajectory is not feasible）应明显变少或消失"
