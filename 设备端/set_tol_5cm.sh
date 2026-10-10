#!/bin/bash
# =====================================================================
# 把导航的【停位容差】改成 学长A论文里的 5cm 标准
#   xy_goal_tolerance  0.1  → 0.05    （位置 ±5cm，对齐论文口径）
#   yaw_goal_tolerance 0.2  → 0.05    （朝向 ±2.9°，为固定姿态抓取收严）
#
# 用法（小车上）：
#     bash set_tol_5cm.sh
#
# 为什么 yaw 也要收：
#   机械臂 grap 是固定姿态。车朝向歪 0.1rad≈5.7° 时，机械臂伸出 20cm
#   末端就横偏 ≈2cm —— 3cm 的物块直接夹空 ✗
#
# 改完【必须重启导航】（move_base 只在启动时读一次参数 ✗）
# 想回滚：同目录下 *.bak5cm 就是原文件 → cp xxx.bak5cm xxx
# =====================================================================
set -u

PKG=$(rospack find wd_robot_navigation 2>/dev/null)
if [ -z "$PKG" ]; then
    echo "✗ 找不到 wd_robot_navigation 包（先 source ~/catkin_ws/devel/setup.bash）"
    exit 1
fi
echo "包路径: $PKG"
cd "$PKG" || exit 1

FILES=$(grep -rl "goal_tolerance" . \
        --include=*.launch --include=*.yaml --include=*.yml 2>/dev/null)
if [ -z "$FILES" ]; then
    echo "✗ 在这个包里没找到配 goal_tolerance 的文件"
    echo "  手动找找： grep -rn goal_tolerance $PKG"
    exit 1
fi

echo
echo "=== 改前 ==="
grep -rn "goal_tolerance" . --include=*.launch --include=*.yaml --include=*.yml 2>/dev/null

for f in $FILES; do
    [ -f "$f.bak5cm" ] || cp "$f" "$f.bak5cm"     # 只在第一次备份
    # 同时兼容 YAML 写法（xxx: 0.1）和 launch 写法（name="xxx" value="0.1"）
    sed -i \
      -e 's/\(xy_goal_tolerance:[[:space:]]*\)[0-9.]*/\10.05/' \
      -e 's/\(yaw_goal_tolerance:[[:space:]]*\)[0-9.]*/\10.05/' \
      -e 's/\(name="xy_goal_tolerance"[^>]*value="\)[0-9.]*/\10.05/' \
      -e 's/\(name="yaw_goal_tolerance"[^>]*value="\)[0-9.]*/\10.05/' \
      "$f"
    echo "  ✔ 已改 $f   （原文件备份: $f.bak5cm）"
done

echo
echo "=== 改后（应该都是 0.05）==="
grep -rn "goal_tolerance" . --include=*.launch --include=*.yaml --include=*.yml 2>/dev/null

echo
echo "★ 下一步：Ctrl+C 停掉导航，重新起一次（参数只在启动时读 ✗）"
echo "  source ~/catkin_ws/devel/setup.bash && roslaunch wd_robot_navigation robot_navigation.launch open_rviz:=true map_file:=\$HOME/chen_car_drone/maps/lab_20261009.yaml"
echo "★ 起完确认生效： rosparam get /move_base/TebLocalPlannerROS/xy_goal_tolerance   → 应该是 0.05"
