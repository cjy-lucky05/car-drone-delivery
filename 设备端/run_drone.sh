#!/bin/bash
# ============================================================
# run_drone.sh —— NX（无人机端）一键运行脚本
# 放在 /mnt/usb/chenjiayu/无人机/ 下
#
# 用法：
#   bash run_drone.sh                       # 常驻（收平台指令）
#   bash run_drone.sh slot 1                # 问库位（站点1）
#   bash run_drone.sh drop 1 1A 2           # 上报投放：站点1 库位1A 2件
#   bash run_drone.sh self                  # 自检
# ============================================================

DIR=/mnt/usb/chenjiayu/无人机
cd "$DIR" || { echo "✗ 找不到 $DIR"; exit 1; }

case "$1" in
  slot)
    SITE=${2:-1}
    python3 device_client.py --role drone --event ask_slot --site "$SITE"
    ;;
  drop)
    SITE=${2:-1}; SLOT=${3:-1A}; CNT=${4:-1}
    python3 device_client.py --role drone --event drop_done --site "$SITE" \
      --extra "{\"slot\":\"$SLOT\",\"count\":$CNT}"
    ;;
  self)
    python3 device_client.py --role drone --test
    ;;
  ""|listen)
    echo "[NX] 常驻模式，收平台指令，Ctrl+C 退出"
    python3 device_client.py --role drone
    ;;
  *)
    echo "用法："
    echo "  bash run_drone.sh                 # 常驻收指令"
    echo "  bash run_drone.sh slot 1          # 问库位"
    echo "  bash run_drone.sh drop 1 1A 2     # 上报投放（站点1 库位1A 2件）"
    echo "  bash run_drone.sh self            # 自检"
    ;;
esac
