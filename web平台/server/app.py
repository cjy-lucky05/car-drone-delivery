# -*- coding: utf-8 -*-
"""
车-机协同末端配送 · 网页监控台（FastAPI 后端）
==============================================
职责（★ 只做两件事，不做决策）：
  ① 读数据库 → 提供 HTTP 接口给网页（库位/任务/事件/统计）
  ② 把人的操作转成 MQTT 消息发出去 → 由 hub.py 决策并写库

★ 为什么前端不直接写库：
  保持 hub.py 是【唯一写入者】= 单一真相源，零并发写问题 ✓

启动：
    cd web平台/server && python3 -u app.py
    或  uvicorn app:app --host 0.0.0.0 --port 8000
浏览器访问：http://<服务器IP>:8000
接口文档：  http://<服务器IP>:8000/docs   ← FastAPI 自带，可点着测
"""
import os
import sys
from typing import Any, Dict, List, Optional

# 依赖目录（各机器放在项目 libs/ 下）
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, "..", "..", "libs"),
           os.path.join(_HERE, "..", "libs"),
           os.path.expanduser("~/libs")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from fastapi import FastAPI, Query
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.responses import FileResponse, JSONResponse
    from pydantic import BaseModel
except ImportError:
    print("缺少 fastapi。安装：pip3 install --target=<项目>/libs fastapi 'uvicorn[standard]'")
    raise

import db
import mqtt_cmd

STATIC = os.path.join(_HERE, "..", "static")
NS = mqtt_cmd.NS

app = FastAPI(title="车-机协同末端配送 · 监控台", version="1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


# ------------------------------------------------------------------ 页面
@app.get("/")
def index():
    return FileResponse(os.path.join(STATIC, "index.html"))


# ------------------------------------------------------------------ 数据接口
@app.get("/api/status")
def api_status():
    """★ 网页轮询这一个接口就够了：一次拿全"""
    tasks = db.list_tasks(limit=20)
    return {
        "devices": db.list_devices(),
        "slots": db.list_slots(),
        "tasks": tasks,
        "queue": db.list_queue(),
        "queue_size": db.queue_size(),
        "stats": db.stats(),
    }


@app.get("/api/tasks")
def api_tasks(limit: int = 50, site: Optional[str] = None, status: Optional[str] = None):
    return db.list_tasks(limit=limit, site=site, status=status)


@app.get("/api/events")
def api_events(limit: int = 100, device: Optional[str] = None):
    return db.list_events(limit=limit, device=device)


@app.get("/api/stats")
def api_stats():
    """★ 论文用的统计：完成数 / 成功率 / 平均耗时"""
    return db.stats()


@app.get("/api/slots")
def api_slots():
    return db.list_slots()


# ------------------------------------------------------------------ 下发指令
class CmdIn(BaseModel):
    target: str                 # 设备 id，如 car-01 / nx-01
    cmd: str                    # 指令名，如 goto_pick / goto_deliver / start
    site: str = ""
    slot: str = ""
    task_id: str = ""
    count: int = 1


@app.post("/api/cmd")
def api_cmd(body: CmdIn):
    """把人的操作发成 MQTT 消息（★ 不直接写库）"""
    extra: Dict[str, Any] = {}
    if body.site:
        extra["site"] = body.site
    if body.slot:
        extra["slot"] = body.slot
    if body.task_id:
        extra["task_id"] = body.task_id
    extra["count"] = body.count
    if body.target.startswith("nx"):
        r = mqtt_cmd.send_to_drone(body.target, body.cmd, **extra)
    else:
        r = mqtt_cmd.send_to_car(body.target, body.cmd, **extra)
    # 记一笔"人下的令"到事件表（便于追溯）
    if r.get("ok"):
        db.log_event("web", "cmd_" + body.cmd, {"target": body.target, **extra})
    return JSONResponse(r)


@app.get("/api/health")
def api_health():
    return {"ok": True, "broker": "%s:%s" % (mqtt_cmd.BROKER, mqtt_cmd.PORT),
            "has_cred": bool(mqtt_cmd.USERNAME and mqtt_cmd.PASSWORD)}


if __name__ == "__main__":
    import uvicorn
    db.init_db()
    print("=" * 58)
    print("  车-机协同末端配送 · 监控台")
    print("  数据库:", os.path.abspath(db.DB_PATH))
    print("  浏览器访问:  http://<服务器IP>:8000")
    print("  接口文档:    http://<服务器IP>:8000/docs")
    print("=" * 58)
    uvicorn.run(app, host="0.0.0.0", port=8000)
