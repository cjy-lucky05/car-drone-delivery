#!/bin/bash
# 停止网页监控台（★ 只停本项目，不碰别人的服务）
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER="$(dirname "$HERE")/server"
if [ -f "$HERE/web.pid" ]; then
    pid=$(cat "$HERE/web.pid")
    kill "$pid" 2>/dev/null && echo "  - 已停 web (pid $pid)"
    rm -f "$HERE/web.pid"
fi
# ★★ 不用 pkill -f（ssh 命令行含同样字符串会自杀）→ 先查 PID 再 kill
for _p in $(ps -eo pid,cmd --no-headers | grep "$SERVER/app.py" | grep -v grep | awk '{print $1}'); do
    echo "  - 停 进程 $_p"; kill "$_p" 2>/dev/null
done
sleep 1
left=$(ps -ef | grep "$SERVER/app.py" | grep -v grep | wc -l)
[ "$left" = "0" ] && echo "  ✅ 已停止" || echo "  ⚠️ 还剩 $left 个进程"
