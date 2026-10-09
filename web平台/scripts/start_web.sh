#!/bin/bash
# ============================================================================
#  车-机协同末端配送 · 网页监控台【一键启动】
# ============================================================================
#  用法：
#      bash start_web.sh            # 后台启动（日志写到 logs/web.log）
#      bash start_web.sh -f         # 前台启动（直接看日志，调试用）
#      bash stop_web.sh             # 停止
# ============================================================================
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB="$(dirname "$HERE")"
SERVER="$WEB/server"

# 依赖目录（服务器上是 /root/chenjiayu/libs；其他机器放项目 libs/）
for d in "/root/chenjiayu/libs" "$WEB/libs" "$HOME/libs"; do
    if [ -d "$d" ]; then export PYTHONPATH="$d:${PYTHONPATH:-}"; break; fi
done

LOG_DIR="/root/chenjiayu/logs"
[ -d "$LOG_DIR" ] || LOG_DIR="$WEB/logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/web.log"

# 检查密码配置
if [ ! -f "$SERVER/config_local.py" ]; then
    echo "⚠️  缺少 config_local.py（放 MQTT 密码用，不进仓库）"
    echo "    解决: cp $SERVER/config.example.py $SERVER/config_local.py"
    echo "    然后编辑，把密码填进去"
    exit 1
fi

cd "$SERVER" || exit 1

if [ "${1:-}" = "-f" ]; then
    echo "前台启动（Ctrl+C 退出）… 浏览器访问 http://<服务器IP>:8000"
    exec python3 -u "$SERVER/app.py"
fi

# 后台启动（先停旧的）
# ★★ 不用 pkill -f！—— ssh 命令行里含同样的字符串时会把自己也杀掉（实测踩过）
#    改成：先查出 PID，再逐个 kill
for _p in $(ps -eo pid,cmd --no-headers | grep "$SERVER/app.py" | grep -v grep | awk '{print $1}'); do
    kill "$_p" 2>/dev/null
done
sleep 1
nohup python3 -u "$SERVER/app.py" > "$LOG" 2>&1 &
echo $! > "$HERE/web.pid"
sleep 3

echo "=============================================================="
echo "  ✅ 网页监控台已启动"
echo "=============================================================="
echo "  浏览器访问：  http://<服务器IP>:${WEB_PORT:-8010}   ★ 端口用 8010（8000/8001 是别人的）"
echo "  接口文档：    http://<服务器IP>:${WEB_PORT:-8010}/docs   ← 可点着测"
echo "  实时日志：    tail -f $LOG"
echo "  停止服务：    bash $HERE/stop_web.sh"
echo "--------------------------------------------------------------"
tail -n 12 "$LOG" 2>/dev/null || true
echo "=============================================================="
