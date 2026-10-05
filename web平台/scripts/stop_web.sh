#!/bin/bash
# 停止网页监控台
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$HERE/web.pid" ]; then
    pid=$(cat "$HERE/web.pid")
    kill "$pid" 2>/dev/null && echo "  - 已停 web (pid $pid)"
    rm -f "$HERE/web.pid"
fi
pkill -f "app.py" 2>/dev/null
sleep 1
left=$(ps -ef | grep "app.py" | grep -v grep | wc -l)
[ "$left" = "0" ] && echo "  ✅ 已停止" || echo "  ⚠️ 还剩 $left 个进程"
