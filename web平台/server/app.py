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
import random
import re
import sys
import time
import json
from typing import Any, Dict, List, Optional

# 依赖目录（各机器放在项目 libs/ 下）
_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.join(_HERE, "..", "..", "libs"),      # 开发机
           os.path.join(_HERE, "..", "..", "..", "libs"), # 服务器: /root/chenjiayu/libs
           os.path.join(_HERE, "..", "libs"),
           "/root/chenjiayu/libs",
           os.path.expanduser("~/libs")):
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from fastapi import FastAPI, Query, Request, Depends, HTTPException
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.responses import FileResponse, JSONResponse, Response
    from fastapi.staticfiles import StaticFiles
    from pydantic import BaseModel
except ImportError:
    print("缺少 fastapi。安装：pip3 install --target=<项目>/libs fastapi 'uvicorn[standard]'")
    raise

import db
import mqtt_cmd

STATIC = os.path.join(_HERE, "..", "static")
NS = mqtt_cmd.NS

# ★ 设备离线判定：超过这么多秒没上报 → 视为离线（MQTT 未配 LWT 时的近似做法）
DEV_OFFLINE_AFTER = int(os.environ.get("DEV_OFFLINE_AFTER", "60"))

# ============================================================================
#  ★★★ 登录态（token）—— 2026-10-09 新增
#  之前所有接口【谁都能调】：删别人包裹、伪造别人手机号查包裹、伪造无人机位置……
#  现在：登录发一张 token，前端每次请求带上；后端校验，不通过直接 401/403
#  token 存内存（重启失效 → 重新登录即可；毕设规模够用）
# ============================================================================
import secrets as _secrets
import threading as _threading

TOKEN_TTL = int(os.environ.get("TOKEN_TTL", str(24 * 3600)))   # 24 小时
_TOKENS: Dict[str, Dict[str, Any]] = {}
_tok_lock = _threading.Lock()


def _new_token(u: Dict[str, Any]) -> str:
    """★ 给登录成功的用户发一张 token"""
    t = _secrets.token_urlsafe(24)
    with _tok_lock:
        _TOKENS[t] = {"user_id": u.get("user_id"), "name": u.get("name"),
                      "phone": u.get("phone") or "", "role": u.get("role") or "user",
                      "expire": time.time() + TOKEN_TTL}
        # 顺手清理过期 token，防止内存无限涨
        if len(_TOKENS) > 500:
            _now2 = time.time()
            for k in [k for k, v in _TOKENS.items() if v.get("expire", 0) < _now2]:
                _TOKENS.pop(k, None)
    return t


def _who(request: Request) -> Optional[Dict[str, Any]]:
    """★ 从请求头取 token → 查出是谁（没有/过期返回 None）"""
    tok = (request.headers.get("X-Token") or "").strip()
    if not tok:
        au = (request.headers.get("Authorization") or "").strip()
        if au.lower().startswith("bearer "):
            tok = au[7:].strip()
    if not tok:
        return None
    with _tok_lock:
        u = _TOKENS.get(tok)
        if not u:
            return None
        if u.get("expire", 0) < time.time():
            _TOKENS.pop(tok, None)
            return None
        return dict(u)


def _require_login(request: Request) -> Dict[str, Any]:
    """依赖：必须是【已登录】用户"""
    u = _who(request)
    if not u:
        raise HTTPException(status_code=401, detail="未登录或登录已过期，请重新登录")
    return u


def _require_roles(*roles: str):
    """依赖工厂：必须是【指定角色】之一"""
    def _dep(request: Request) -> Dict[str, Any]:
        u = _require_login(request)
        if u.get("role") not in roles:
            raise HTTPException(status_code=403,
                                detail="没有权限（需要 %s）" % " / ".join(roles))
        return u
    return _dep


# ★ 常用的三个档
_dep_user = _require_login                              # 登录即可（用户级）
_dep_staff = _require_roles("admin", "drone_staff")      # 管理 / 工作人员
_dep_admin = _require_roles("admin")                     # 仅超级管理员


# ============================================================================
#  ★★★ 离线保护（2026-10-09）
#  语义：设备【在线】= 它的机载程序最近 60 秒内有上报。
#        离线设备不接受网页操作（点了也发不过去，只会造成"假在线"误导）✗
#  临时演示（设备不在手边）可关掉：REQUIRE_ONLINE=0
# ============================================================================
REQUIRE_ONLINE = os.environ.get("REQUIRE_ONLINE", "1").strip() not in ("0", "false", "False", "no")


def _is_online(device_id: str) -> bool:
    """★ 这台设备在线吗（最近 60 秒内有上报）"""
    d = db.get_device(device_id)
    if not d:
        return False
    ls = int(d.get("last_seen") or 0)
    return bool(ls) and ((int(time.time()) - ls) <= DEV_OFFLINE_AFTER)


def _offline_msg(device_id: str) -> str:
    return ("%s 当前离线，不能操作。\n"
            "在线 = 机载程序正在上报（最近 60 秒内有消息）。\n"
            "请先启动它：无人机 → 在机载电脑跑 device_client.py --role drone；"
            "小车 → 启动桥接节点 mqtt_bridge_node.py" % device_id)

# ★ 包裹状态/事件的中文名（和 hub.py 保持一致；hub 不在同一进程，这里独立定义）
PARCEL_CN = {
    "CREATED": "已揽收", "ON_DRONE": "无人机运输中", "AT_GATE": "已到校门口站点",
    "LOADED": "小车已装载", "DELIVERING": "校园配送中",
    "PLACED": "已放入货柜(待取)", "PICKED": "已取件",
}
PARCEL_EV_CN = {
    "parcel_in":    "中转点揽收挂载",
    "drone_depart": "无人机起飞",
    "drone_arrive": "无人机到达校门口站点",
    "car_load":     "小车装载（扫码识别）",
    "car_depart":   "小车出发配送",
    "dst_arrive":   "到达目的地",
    "shelf_place":  "放入货柜(低层)",
    "picked_up":    "扫码取件（已签收）",
    # ★★★ 2026-10-09 补全（之前漏了这些，时间线上直接显示英文 ✗）
    "set_dest":     "修改送达地点",
    "parcel_add":   "手动添加包裹",
    "parcel_del":   "删除包裹",
    "drop_done":    "已放货到校门口站点",
    "drone_takeoff":"无人机起飞",
    "drone_loaded": "已取货（挂载）",
    "takeoff":      "无人机起飞",
    "loaded":       "已取货（挂载）",
    "ask_slot":     "询问库位",
    "use_slot":     "平台分配库位",
    "wait":         "原地等待",
    "pick":         "开始抓取",
    "goto_pick":    "前往取货点",
    "goto_deliver": "前往送达点",
    "arrived":      "到达取货点",
    "pick_done":    "取货完成",
    "delivered":    "送达完成",
    "go_home":      "返回起始点",
    "back_home":    "已回到起点",
    "idle":         "空闲待命",
    "busy":         "任务执行中",
    "exception":    "设备异常",
    "selftest":     "设备自检",
    "heartbeat":    "心跳保活",
    "position":     "位置上报",
    "gps":          "GPS位置上报",
    "battery":      "电量上报",
}

app = FastAPI(title="车-机协同末端配送 · 监控台", version="1.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# ★ 挂载第三方静态库（Leaflet 地图库，自托管不依赖 CDN）
_LIBDIR = os.path.join(STATIC, "lib")
if os.path.isdir(_LIBDIR):
    app.mount("/lib", StaticFiles(directory=_LIBDIR), name="lib")


@app.get("/config_local.js")
def _map_cfg_js():
    """★★★ 场地私有地图配置（经纬度）—— 文件【不进 git】，每台机器各放一份

    - 存在 → 返回它，前端用它覆盖 index.html 里的占位坐标 ✓
    - 不存在 → 返回空注释，前端自动退回占位值（不报错）✓
    """
    p = os.path.join(STATIC, "config_local.js")
    if os.path.isfile(p):
        return FileResponse(p, media_type="application/javascript",
                            headers={"Cache-Control": "no-cache"})
    return Response("// config_local.js 不存在 —— 前端用 index.html 里的占位坐标\n",
                    media_type="application/javascript",
                    headers={"Cache-Control": "no-cache"})


# ------------------------------------------------------------------ 页面
@app.get("/")
def index():
    """★ 首页 —— 加 no-cache 头，改完前端不用再强刷（Ctrl+F5）"""
    return FileResponse(os.path.join(STATIC, "index.html"),
                        headers={"Cache-Control": "no-cache, no-store, must-revalidate",
                                 "Pragma": "no-cache", "Expires": "0"})


# ------------------------------------------------------------------ 数据接口
# ★★★ /api/status 结果缓存（2026-10-09）：削峰
#   多少用户都在 2 秒轮询，1 秒内也只查一次库 → 用户数涨了后端压力不涨 ✓
STATUS_CACHE_TTL = float(os.environ.get("STATUS_CACHE_TTL", "1.0"))
_status_cache: Dict[str, Any] = {"ts": 0.0, "data": None}
_status_lock = _threading.Lock()


@app.get("/api/status")
def api_status(_u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 网页轮询这一个接口就够了：一次拿全（结果缓存 1 秒）"""
    _nowc = time.time()
    with _status_lock:
        c = _status_cache
        if c["data"] is not None and (_nowc - c["ts"]) < STATUS_CACHE_TTL:
            return c["data"]
    data = _build_status()
    with _status_lock:
        _status_cache["ts"] = time.time()
        _status_cache["data"] = data
    return data


def _build_status():
    tasks = db.list_tasks(limit=20)
    # ★ 按 last_seen 判定在线/离线 —— 否则设备关机后网页永远显示“空闲”
    devs = db.list_devices()
    _now = int(time.time())
    for d in devs:
        ls = int(d.get("last_seen") or 0)
        d["age_sec"] = (_now - ls) if ls else None
        if (not ls) or (_now - ls > DEV_OFFLINE_AFTER):
            d["status"] = "offline"
    return {
        "devices": devs,
        "slots": db.list_slots(),
        "tasks": tasks,
        "queue": db.list_queue(),
        "queue_size": db.queue_size(),
        "stats": db.stats(),
    }


@app.get("/api/tasks")
def api_tasks(limit: int = 50, site: Optional[str] = None, status: Optional[str] = None, _u: Dict[str, Any] = Depends(_dep_staff)):
    return db.list_tasks(limit=limit, site=site, status=status)


@app.get("/api/parcels")
def api_parcels(limit: int = 200, status: str = "", car_id: str = "", q: str = "", _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 包裹列表；q = 关键词跨字段搜索（服务端搜，包裹再多也能搜到）"""
    rows = db.list_parcels(limit=limit, status=status or None,
                           car_id=car_id or None, q=q or None)
    for r in rows:
        r["status_cn"] = PARCEL_CN.get(r.get("status"), r.get("status"))
    return {"ok": True, "parcels": rows, "stats": db.parcel_stats()}


@app.get("/api/parcel/{parcel_id}")
def api_parcel_detail(parcel_id: str, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 单件包裹详情 + 【它的完整时间线】（前端主画面用）"""
    p = db.get_parcel(parcel_id)
    if not p:
        return JSONResponse({"ok": False, "detail": "没有这件包裹：%s" % parcel_id})
    evs = db.list_events(parcel_id=parcel_id, limit=200, asc=True)
    tl = []
    for e in evs:
        detail = ""
        try:
            detail = (json.loads(e.get("payload") or "{}") or {}).get("detail", "")
        except Exception:
            detail = e.get("payload") or ""
        tl.append({"event": e.get("event"), "event_cn": PARCEL_EV_CN.get(e.get("event"), e.get("event")),
                   "detail": detail, "actor": e.get("device"), "ts": e.get("ts")})
    p["status_cn"] = PARCEL_CN.get(p.get("status"), p.get("status"))
    return {"ok": True, "parcel": p, "timeline": tl}


@app.get("/api/parcel/track/{tracking_no}")
def api_parcel_track(tracking_no: str, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 用【快递单号】查包裹 —— 模拟机械臂扫到条码后查库"""
    p = db.find_parcel(tracking_no=tracking_no)
    if not p:
        return JSONResponse({"ok": False, "detail": "库里没有这个运单号：%s" % tracking_no})
    return api_parcel_detail(p["parcel_id"])


@app.get("/api/my")
def api_my(q: str = "", phone: str = "", _u: Dict[str, Any] = Depends(_dep_user)):
    """
    ★★ 用户视角：只查【我自己的】包裹（按手机号 / 取件码 / 姓名）
       和 /api/parcel/{id} 的区别：这里【不会】返回别人的数据，也【不暴露】全局信息
    """
    # ★★★ 一律以【登录态里的手机号】为准（不信任前端传的 phone → 防越权查别人）
    if _u.get("role") == "admin" and (q or phone):
        q = (q or phone or "").strip()          # 管理员：可以按任意条件查
    else:
        q = (_u.get("phone") or "").strip()     # 其他人：只能查自己
    if not q:
        return JSONResponse({"ok": False, "detail": "请输入手机号 / 取件码 / 姓名"})
    # ★ 智能判断输入类型
    digits = "".join(ch for ch in q if ch.isdigit())
    kw = {}
    if len(digits) == 4 and len(q) <= 6:
        kw["pick_code"] = digits            # 4 位 → 取件码
    elif len(digits) >= 7:
        kw["phone"] = digits                # 7 位以上 → 手机号（支持尾号模糊）
    else:
        kw["owner"] = q                     # 其它 → 按姓名
    rows = db.find_parcels_by(**kw)
    out = []
    for p in rows:
        p["status_cn"] = PARCEL_CN.get(p.get("status"), p.get("status"))
        evs = db.list_events(parcel_id=p["parcel_id"], limit=100, asc=True)
        tl = []
        for e in evs:
            try:
                det = (json.loads(e.get("payload") or "{}") or {}).get("detail", "")
            except Exception:
                det = ""
            tl.append({"event": e.get("event"),
                       "event_cn": PARCEL_EV_CN.get(e.get("event"), e.get("event")),
                       "detail": det, "ts": e.get("ts")})
        p["timeline"] = tl
        # ★★ 承运车的【实时位置】—— 用户能在地图上看到"我的货走到哪了"
        #    注意：只给位置，不给设备号（不暴露内部信息）
        cid = p.get("car_id")
        if cid and p.get("status") in ("LOADED", "DELIVERING"):
            try:
                for dv in db.list_devices():
                    if dv.get("device_id") == cid and dv.get("pos_x") is not None:
                        p["vehicle"] = {"x": dv.get("pos_x"), "y": dv.get("pos_y"),
                                        "yaw": dv.get("pos_yaw"), "ts": dv.get("pos_ts")}
                        break
            except Exception:
                pass
        # ★ 用户视角不暴露设备号等内部信息
        p.pop("car_id", None); p.pop("drone_id", None)
        out.append(p)
    if not out:
        return JSONResponse({"ok": False, "detail": "没查到包裹，请核对手机号或取件码"})
    return {"ok": True, "query": q, "parcels": out, "count": len(out)}


# ==================== ★★★ 用户管理（管理员） ====================
@app.get("/api/users")
def api_users(building: str = "", _u: Dict[str, Any] = Depends(_dep_admin)):
    """用户列表（带每人的包裹统计）+ 按楼栋分组"""
    # ★★ 前端下拉的"(未填楼栋)" → 用 __none__ 表示"筛没填楼栋的人"
    #    （别的历史写法统一归到 __none__，避免又被当成"字面楼栋"查不到）
    if building in ("(未填楼栋)", "未填楼栋", "__none__"):
        building = "__none__"
    elif building in ("(空)", "null", "None", "undefined", "all", "全部楼栋"):
        building = ""
    us = db.list_users(building=building or None)
    for u in us:                       # ★ 不把密码散列发给前端（安全）
        u["has_pwd"] = bool(u.pop("password", None))
    if not us:                       # 表空 → 用包裹里的收件人"自动发现"一批，供一键导入
        seen, auto = set(), []
        for p in db.list_parcels(limit=500):
            nm, ph = p.get("owner"), p.get("phone")
            if nm and (nm, ph) not in seen:
                seen.add((nm, ph)); auto.append({"name": nm, "phone": ph})
        return {"ok": True, "users": db.list_users_agg(), "groups": [],
                "suggest": auto, "empty": True}
    return {"ok": True, "users": us, "groups": db.users_grouped()}


@app.post("/api/users")
async def api_user_create(req: Request, _u: Dict[str, Any] = Depends(_dep_admin)):
    """新增用户 {name, phone, building, note, role}"""
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    name = str(b.get("name") or "").strip()
    phone = str(b.get("phone") or "").strip()
    if not name or not phone:
        return JSONResponse({"ok": False, "detail": "姓名和手机号必填"})
    if db.get_user_by_phone(phone):
        return JSONResponse({"ok": False, "detail": "手机号 %s 已存在" % phone})
    uid = db.upsert_user(name=name, phone=phone, building=str(b.get("building") or ""),
                         role=str(b.get("role") or "user"), note=str(b.get("note") or ""))
    return {"ok": bool(uid), "user_id": uid}


@app.put("/api/users/{user_id}")
async def api_user_update(user_id: str, req: Request, _u: Dict[str, Any] = Depends(_dep_admin)):
    """修改用户"""
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    o = db.get_user(user_id)
    if not o:
        return JSONResponse({"ok": False, "detail": "用户不存在：%s" % user_id})
    merged = {
        "name": b.get("name", o.get("name") or ""),
        "phone": b.get("phone", o.get("phone") or ""),
        "building": b.get("building", o.get("building") or ""),
        "role": b.get("role", o.get("role") or "user"),
        "note": b.get("note", o.get("note") or ""),
        "active": int(b.get("active", o.get("active", 1) or 1)),
    }
    if merged["phone"] != (o.get("phone") or ""):
        dup = db.get_user_by_phone(merged["phone"])
        if dup and dup.get("user_id") != user_id:
            return JSONResponse({"ok": False, "detail": "手机号已被占用"})
    db.upsert_user(user_id=user_id, **merged)
    return {"ok": True, "user": db.get_user(user_id)}


@app.delete("/api/users/{user_id}")
def api_user_delete(user_id: str, _u: Dict[str, Any] = Depends(_dep_admin)):
    n = db.delete_user(user_id)
    return {"ok": bool(n), "deleted": n}


# ==================== ★★★ 用户登录（手机号 + 验证码）====================
# 毕设级：验证码存内存（够用）。生产应接短信网关 + Redis，论文里如实写"模拟短信"
_CODES = {}          # {phone: (code, expire_ts)}
_CODE_TTL = 300


@app.get("/api/send_code")
def api_send_code(phone: str = ""):
    """发送验证码（演示模式：直接在页面上显示，不真发短信）"""
    phone = (phone or "").strip()
    if len("".join(ch for ch in phone if ch.isdigit())) < 7:
        return JSONResponse({"ok": False, "detail": "请输入正确的手机号"})
    u = db.get_user_by_phone(phone)
    if not u:
        return JSONResponse({"ok": False, "detail": "该手机号未注册，请联系管理员开通"})
    if not (u.get("active", 1) or 0):
        return JSONResponse({"ok": False, "detail": "账号已停用"})
    code = "%06d" % random.randint(0, 999999)
    _CODES[phone] = (code, time.time() + _CODE_TTL)
    print("[平台] 验证码 %s → %s（%s）" % (code, phone, u.get("name")))
    return {"ok": True, "detail": "验证码已生成（演示模式直接显示）",
            "code": code, "ttl": _CODE_TTL, "name": u.get("name")}


@app.post("/api/login_pwd")
async def api_login_pwd(req: Request):
    """★★ 账号密码登录（admin/admin123 · user/user123）；account 可填用户名或手机号"""
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    acct = str(b.get("account") or b.get("phone") or "").strip()
    pw = str(b.get("password") or "")
    if not acct or not pw:
        return JSONResponse({"ok": False, "detail": "账号和密码都要填"})
    u = db.login_by_account(acct, pw)
    if not u:
        return JSONResponse({"ok": False, "detail": "账号或密码不正确"})
    print("[平台] 登录成功：%s（%s）" % (u.get("name"), u.get("role")))
    _usr = {"user_id": u.get("user_id"), "name": u.get("name"),
            "phone": u.get("phone"), "building": u.get("building"),
            "role": u.get("role") or "user"}
    return {"ok": True, "user": _usr, "token": _new_token(_usr)}   # ★ 发 token


@app.post("/api/register")
async def api_register(req: Request):
    """★★ 用户注册（注册出来的都是普通用户 role=user）"""
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    name = str(b.get("name") or "").strip()
    phone = str(b.get("phone") or "").strip()
    pw = str(b.get("password") or "")
    bld = str(b.get("building") or "").strip()
    if not name or not phone or not pw:
        return JSONResponse({"ok": False, "detail": "姓名、手机号、密码都要填"})
    if not re.match(r"^1\d{10}$", phone):        # ★ 手机号必须是 11 位数字
        return JSONResponse({"ok": False, "detail": "手机号格式不对（应为 11 位数字）"})
    if len(pw) < 6:
        return JSONResponse({"ok": False, "detail": "密码至少 6 位"})
    # ★ 保留名：不能占用系统账号名（否则界面上会分不清谁是管理员）
    if name.strip() in ("管理员", "普通用户", "admin", "user", "administrator", "root"):
        return JSONResponse({"ok": False, "detail": "「%s」是系统保留名，请换一个" % name.strip()})
    if db.get_user_by_phone(phone):
        return JSONResponse({"ok": False, "detail": "该手机号已注册，请直接登录"})
    dup = db.find_by_name(name)
    warn = ""
    if dup:
        warn = "（已有 %d 位同名用户，不影响使用 —— 登录是用手机号的）" % len(dup)
    conn_uid = db.upsert_user(name=name, phone=phone, building=bld, role="user")
    if not conn_uid:
        return JSONResponse({"ok": False, "detail": "注册失败，请重试"})
    db.set_password(conn_uid, pw)
    u = db.get_user(conn_uid)
    print("[平台] 新用户注册：%s / %s%s" % (name, phone, warn))
    return {"ok": True, "warn": warn,
            "user": {"user_id": u.get("user_id"), "name": u.get("name"),
                     "phone": u.get("phone"), "building": u.get("building"),
                     "role": "user"}}


@app.post("/api/user_setpwd")
async def api_user_setpwd(req: Request, _u: Dict[str, Any] = Depends(_dep_admin)):
    """★★ 管理员给某用户重置密码 {user_id, password}"""
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    uid = str(b.get("user_id") or "").strip()
    pw = str(b.get("password") or "")
    if not uid or not pw:
        return JSONResponse({"ok": False, "detail": "缺少用户或密码"})
    if len(pw) < 6:
        return JSONResponse({"ok": False, "detail": "密码至少 6 位"})
    u = db.get_user(uid)
    if not u:
        return JSONResponse({"ok": False, "detail": "用户不存在"})
    db.set_password(uid, pw)
    print("[平台] 管理员重置密码：%s（%s）" % (u.get("name"), uid))
    return {"ok": True, "detail": "已给「%s」设置新密码" % u.get("name")}


@app.post("/api/find_pwd")
async def api_find_pwd(req: Request):
    """
    ★★★ 找回密码：只【查出并显示原密码】，不做任何修改
    验证：手机号 + （姓名 或 楼栋 任意一项）
    """
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    phone = str(b.get("phone") or b.get("account") or "").strip()
    nm = str(b.get("name") or "").strip()
    bld = str(b.get("building") or "").strip()
    if not phone:
        return JSONResponse({"ok": False, "detail": "请填手机号"})
    if not nm and not bld:
        return JSONResponse({"ok": False, "detail": "再填【姓名】或【宿舍楼号】任意一项来验证身份"})

    def _num(s):
        return "".join(c for c in str(s) if c.isdigit())

    u = db.get_user_by_phone(phone)
    if not u:
        return JSONResponse({"ok": False, "detail": "这个手机号没注册过，请核对"})
    ok_name = bool(nm) and (nm == (u.get("name") or ""))
    ub = u.get("building") or ""
    ok_bld = bool(bld) and bool(ub) and (_num(ub) == _num(bld))
    if not (ok_name or ok_bld):
        hint = []
        if u.get("name"):
            hint.append("姓名首字：" + u["name"][0] + "…")
        if ub:
            hint.append("登记楼栋：" + ub)
        return JSONResponse({"ok": False,
                             "detail": "验证不通过（" + ("；".join(hint) if hint else "无登记信息") + "）"})

    pw = db.dec_pw(u.get("password"))
    if pw is None:
        return JSONResponse({"ok": False,
                             "detail": "该账号的密码是旧版本存储，取不出来 —— 请改用【重置密码】"})
    print("[平台] 找回密码（只读）：%s（%s）" % (u.get("name"), phone))
    return {"ok": True, "password": pw,
            "detail": "身份验证通过",
            "user": {"user_id": u.get("user_id"), "name": u.get("name"),
                     "phone": u.get("phone"), "building": u.get("building"),
                     "role": u.get("role") or "user"}}


@app.post("/api/reset_pwd2")
async def api_reset_pwd2(req: Request):
    """
    ★★★ 重置密码：手机号 + 【当前密码】验证身份 → 设置新密码
    """
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    phone = str(b.get("phone") or "").strip()
    oldpw = str(b.get("password") or b.get("old_password") or "")
    newpw = str(b.get("new_password") or "")
    if not phone or not oldpw:
        return JSONResponse({"ok": False, "detail": "请填手机号和当前密码"})
    if not newpw:
        return JSONResponse({"ok": False, "detail": "请填新密码"})
    if len(newpw) < 6:
        return JSONResponse({"ok": False, "detail": "新密码至少 6 位"})
    u = db.get_user_by_phone(phone)
    if not u:
        return JSONResponse({"ok": False, "detail": "这个手机号没注册过，请核对"})
    if not db.verify_pw(u.get("password"), oldpw):
        return JSONResponse({"ok": False, "detail": "当前密码不正确"})
    if oldpw == newpw:
        return JSONResponse({"ok": False, "detail": "新密码不能和当前密码一样"})
    db.set_password(u["user_id"], newpw)
    print("[平台] 重置密码：%s（%s）→ 已更新" % (u.get("name"), phone))
    return {"ok": True, "detail": "密码已重置，请用新密码登录",
            "user": {"user_id": u.get("user_id"), "name": u.get("name"),
                     "phone": u.get("phone"), "building": u.get("building"),
                     "role": u.get("role") or "user"}}


@app.post("/api/reset_pwd")
async def api_reset_pwd(req: Request):
    """★★ 忘记密码：手机号 + 验证码 + 新密码"""
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    phone = str(b.get("phone") or "").strip()
    code = str(b.get("code") or "").strip()
    pw = str(b.get("password") or "")
    if not phone or not code or not pw:
        return JSONResponse({"ok": False, "detail": "手机号、验证码、新密码都要填"})
    if len(pw) < 6:
        return JSONResponse({"ok": False, "detail": "新密码至少 6 位"})
    rec = _CODES.get(phone)
    if not rec:
        return JSONResponse({"ok": False, "detail": "请先获取验证码"})
    if time.time() > rec[1]:
        _CODES.pop(phone, None)
        return JSONResponse({"ok": False, "detail": "验证码已过期，请重新获取"})
    if code != rec[0]:
        return JSONResponse({"ok": False, "detail": "验证码不正确"})
    u = db.get_user_by_phone(phone)
    if not u:
        return JSONResponse({"ok": False, "detail": "该手机号未注册"})
    _CODES.pop(phone, None)
    db.set_password(u["user_id"], pw)
    print("[平台] 用户自助改密：%s（%s）" % (u.get("name"), phone))
    return {"ok": True, "detail": "密码已重置，请用新密码登录",
            "user": {"user_id": u.get("user_id"), "name": u.get("name"),
                     "phone": u.get("phone"), "building": u.get("building"),
                     "role": u.get("role") or "user"}}


@app.get("/api/me")
def api_me(phone: str = "", _u: Dict[str, Any] = Depends(_dep_user)):
    """按手机号取用户信息（登录态恢复用）"""
    u = db.get_user_by_phone((phone or "").strip())
    if not u:
        return JSONResponse({"ok": False, "detail": "用户不存在"})
    return {"ok": True, "user": {"user_id": u.get("user_id"), "name": u.get("name"),
                                 "phone": u.get("phone"), "building": u.get("building"),
                                 "role": u.get("role") or "user"}}


@app.post("/api/login")
async def api_login(req: Request):
    """登录：手机号 + 验证码"""
    try:
        b = await req.json()
    except Exception:
        return JSONResponse({"ok": False, "detail": "JSON 解析失败"})
    phone = str(b.get("phone") or "").strip()
    code = str(b.get("code") or "").strip()
    rec = _CODES.get(phone)
    if not rec:
        return JSONResponse({"ok": False, "detail": "请先获取验证码"})
    if time.time() > rec[1]:
        _CODES.pop(phone, None)
        return JSONResponse({"ok": False, "detail": "验证码已过期，请重新获取"})
    if code != rec[0]:
        return JSONResponse({"ok": False, "detail": "验证码不正确"})
    _CODES.pop(phone, None)
    u = db.get_user_by_phone(phone)
    if not u:
        return JSONResponse({"ok": False, "detail": "用户不存在"})
    _usr = {"user_id": u.get("user_id"), "name": u.get("name"),
            "phone": u.get("phone"), "building": u.get("building"),
            "role": u.get("role") or "user"}
    return {"ok": True, "user": _usr, "token": _new_token(_usr)}   # ★ 发 token


@app.get("/api/devices_full")
def api_devices_full(_u: Dict[str, Any] = Depends(_dep_staff)):
    """★★ 设备信息（管理员）：状态/电量/位置/当前任务"""
    now = int(time.time())
    rows = db.list_devices_full()
    for d in rows:
        d["online"] = (now - int(d.get("last_seen") or 0)) <= DEV_OFFLINE_AFTER
        if not d["online"]:
            d["status"] = "offline"
    return {"ok": True, "devices": rows}


@app.get("/api/events")
def api_events(limit: int = 100, device: Optional[str] = None,
               only_biz: int = 1, q: str = "", _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 事件列表。默认 only_biz=1（过滤心跳/位置等噪音）

    ★ 限量上限 1000：防止一次请求把整张表拉出来（数据涨上来会拖垮后端）
    """
    limit = max(1, min(int(limit or 100), 1000))
    return db.list_events(limit=limit, device=device,
                          only_biz=bool(only_biz), q=(q or None))


@app.get("/api/stats")
def api_stats(_u: Dict[str, Any] = Depends(_dep_staff)):
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
def api_cmd(body: CmdIn, _u: Dict[str, Any] = Depends(_dep_admin)):
    """把人的操作发成 MQTT 消息（★ 不直接写库）"""
    extra: Dict[str, Any] = {}
    if body.site:
        extra["site"] = body.site
    if body.slot:
        extra["slot"] = body.slot
    if body.task_id:
        extra["task_id"] = body.task_id
    extra["count"] = body.count
    # ★★★ 离线保护：设备不在线 → 指令发过去也没人收，直接拦掉（避免误解成"已经派了"）
    if REQUIRE_ONLINE and not _is_online(body.target):
        return JSONResponse({"ok": False, "detail": _offline_msg(body.target)})
    if body.target.startswith("nx"):
        r = mqtt_cmd.send_to_drone(body.target, body.cmd, **extra)
    else:
        r = mqtt_cmd.send_to_car(body.target, body.cmd, **extra)
    # 记一笔"人下的令"到事件表（便于追溯）
    if r.get("ok"):
        db.log_event("web", "cmd_" + body.cmd, {"target": body.target, **extra})
    return JSONResponse(r)


# ------------------------------------------------------------------ ★ 地图
MAP_DIR = os.path.join(_HERE, "..", "data")
MAP_SCALE = int(os.environ.get("MAP_SCALE", "4"))     # 固定放大倍数（AUTO_SCALE=0 时用）
AUTO_SCALE = os.environ.get("MAP_AUTO_SCALE", "1") != "0"   # ★ 默认自动
MAP_TARGET_PX = int(os.environ.get("MAP_TARGET_PX", "900"))  # ★ 自动模式下长边目标像素


def _pgm_size(pgm):
    """读 PGM 头的宽高"""
    with open(pgm, "rb") as f:
        f.readline()
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = [int(x) for x in line.split()[:2]]
    return w, h


def _prepare_map():
    """★ 把 map.pgm 处理成【能看清】的图：
         ① 裁掉四周的"未知区域"（灰色空白）—— 客厅地图有效内容只占中间一小块
         ② 等比放大（最近邻，保持像素方块清晰）
         ③ 同步换算新的 origin，保证前端坐标换算公式不用改
       返回 (png_bytes, meta)；处理结果缓存到文件，不用每次请求都重算。
    """
    pgm = os.path.join(MAP_DIR, "map.pgm")
    yml = os.path.join(MAP_DIR, "map.yaml")
    cache_png = os.path.join(MAP_DIR, ".map_cache.png")
    if not (os.path.isfile(pgm) and os.path.isfile(yml)):
        return None, None

    # 缓存：地图/参数没变就直接用
    stamp = "%d-%d-%s-%s-%s" % (os.path.getmtime(pgm), os.path.getmtime(yml),
                                 MAP_SCALE, AUTO_SCALE, MAP_TARGET_PX)
    stamp_f = os.path.join(MAP_DIR, ".map_cache.stamp")
    if os.path.isfile(cache_png) and os.path.isfile(stamp_f) and \
       open(stamp_f).read().strip() == stamp:
        try:
            import json as _j
            meta = _j.load(open(os.path.join(MAP_DIR, ".map_cache.json"), encoding="utf-8"))
            return open(cache_png, "rb").read(), meta
        except Exception:
            pass

    try:
        from PIL import Image
        import io
        import json as _j
        import yaml
        m = yaml.safe_load(open(yml, encoding="utf-8"))
        res = float(m.get("resolution", 0.05))
        org = m.get("origin", [0.0, 0.0, 0.0])
        img = Image.open(pgm).convert("L")
        W0, H0 = img.size

        # ① 裁掉空白 —— 两级策略（地图常是不规则形状，外接矩形里会有大量灰色）
        #    a) 先做"信息掩码"（未知区域=屏蔽）
        mask = img.point(lambda v: 0 if 180 <= v <= 220 else 255)
        #    b) ★ 投影法：按行/列统计信息占比，只保留"真正有内容"的范围
        #       —— 这样能丢掉右上角那种离群的细线，把有效区域裁得更紧
        try:
            col = mask.resize((mask.width, 1), Image.BOX)     # 每列的平均信息量(0~255)
            row = mask.resize((1, mask.height), Image.BOX)    # 每行
            cdat = list(col.getdata())
            rdat = list(row.getdata())

            def _span(dat, th, n):
                idx = [i for i, v in enumerate(dat) if v > th]
                return (idx[0], idx[-1] + 1) if idx else None

            # ★ 两级阈值：先用 10%（滤掉细线/噪点），裁出来太小就退回 3%
            rng = None
            for th in (25, 8, 3):
                c_ = _span(cdat, th, W0)
                r_ = _span(rdat, th, H0)
                if not c_ or not r_:
                    continue
                area = (c_[1] - c_[0]) * (r_[1] - r_[0])
                if area >= W0 * H0 * 0.06:      # 至少占 6% 才认（防裁太小）
                    rng = (c_[0], r_[0], c_[1], r_[1])
                    break
                rng = rng or (c_[0], r_[0], c_[1], r_[1])
            if rng:
                l, u, r, b = rng
            else:
                bb = mask.getbbox()
                l, u, r, b = (bb if bb else (0, 0, W0, H0))
        except Exception:
            bb = mask.getbbox()
            l, u, r, b = (bb if bb else (0, 0, W0, H0))
        #    c) 留一点边距（顺便保证站点坐标不会被裁出图外）
        pad = max(6, int(W0 * 0.06))
        l = max(0, l - pad); u = max(0, u - pad)
        r = min(W0, r + pad); b = min(H0, b + pad)
        img = img.crop((l, u, r, b))

        # ② 放大（最近邻：保持像素方块不糊）
        #    ★ 动态倍数：裁剪后区域大小不一，按"长边约 900px"反推倍数，
        #      这样无论裁得多紧，网页上的地图大小都稳定、看得清。
        if AUTO_SCALE:
            long_side = max(img.width, img.height)
            sc = int(round(MAP_TARGET_PX / float(long_side))) if long_side else MAP_SCALE
            sc = max(2, min(8, sc))              # 限制在 2~8 倍之间
        else:
            sc = MAP_SCALE
        img = img.resize((img.width * sc, img.height * sc), Image.NEAREST)
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        png = buf.getvalue()

        # ③ 新 origin（图像 y 轴朝下，所以 y 要用 H0 - 下边界）
        new_ox = float(org[0]) + l * res
        new_oy = float(org[1]) + (H0 - b) * res
        meta = {"resolution": res / sc,                 # 像素变大 → 每像素代表更小距离
                "origin": [new_ox, new_oy, 0.0],
                "width": img.width, "height": img.height,
                "orig_width": W0, "orig_height": H0,
                "crop": [l, u, r, b], "scale": sc,
                "image_url": "/api/map.png"}

        # 写缓存
        open(cache_png, "wb").write(png)
        open(os.path.join(MAP_DIR, ".map_cache.json"), "w", encoding="utf-8").write(
            _j.dumps(meta, ensure_ascii=False))
        open(stamp_f, "w").write(stamp)
        return png, meta
    except ImportError as e:
        raise RuntimeError("需要 Pillow：pip3 install --target=/root/chenjiayu/libs Pillow（%s）" % e)


@app.get("/api/drone/alloc")
def api_drone_alloc(_u: Dict[str, Any] = Depends(_dep_staff)):
    """★★ 无人机端显示用：平台给无人机【分配了哪个库位】、还空着哪些格
       工作人员放货前看一眼就知道该放哪 ✓"""
    try:
        slots = [x for x in db.list_slots()
                 if str(x.get("kind") or "transit") != "dst"]
        free = [x["slot_id"] for x in slots
                if not x.get("occupied") and str(x["slot_id"]).startswith("1")]
        used = [x["slot_id"] for x in slots if x.get("occupied") and str(x["slot_id"]).startswith("1")]
        return {"ok": True, "free": sorted(free), "used": sorted(used),
                "hint": ("建议放到：" + (sorted(free)[0] if free else "（1 号位已满，等小车取走先）"))}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.get("/api/dst_slots")
def api_dst_slots(_u: Dict[str, Any] = Depends(_dep_user)):
    """★★ 校内配送点货柜（小车放货、收件人来取的地方）
       2号公寓楼下 / 3号公寓楼下 / 图书馆楼下 / 教学楼楼下 / 菜鸟驿站"""
    try:
        slots = db.list_dst_slots()
        # 按点位分组（前端按组渲染）
        groups: Dict[str, List[Dict[str, Any]]] = {}
        for x in slots:
            groups.setdefault(str(x.get("name") or x.get("site") or "其他"), []).append(x)
        return {"ok": True, "count": len(slots), "groups": groups, "slots": slots}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/dst_point/add")
async def api_dst_point_add(request: Request, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 新增一个配送点（如"5号公寓楼下"，默认 3 格）
       body: {name, cells}"""
    try:
        body = await request.json()
    except Exception:
        body = {}
    name = str(body.get("name") or "").strip()
    n = int(body.get("cells") or 3)
    if not name:
        return {"ok": False, "error": "配送点名称必填"}
    try:
        conn = db._connect()
        try:
            db._migrate(conn)
            t = db._now()
            added = []
            for i in range(max(1, min(12, n))):
                sid = "%s-%s" % (name, chr(ord("A") + i))
                if not conn.execute("SELECT slot_id FROM slots WHERE slot_id=?", (sid,)).fetchone():
                    conn.execute("INSERT INTO slots(slot_id, site, occupied, count, task_id,"
                                 " kind, name, updated_at) VALUES(?,?,?,?,?,?,?,?)",
                                 (sid, name, 0, 0, None, "dst", name, t))
                    added.append(sid)
            conn.commit()
        finally:
            conn.close()
        return {"ok": True, "name": name, "added": added}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/dst_point/remove")
async def api_dst_point_remove(request: Request, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 删掉一个配送点（连同它的格子；格子里有货时拒绝）"""
    try:
        body = await request.json()
    except Exception:
        body = {}
    name = str(body.get("name") or "").strip()
    if not name:
        return {"ok": False, "error": "名称必填"}
    try:
        conn = db._connect()
        try:
            db._migrate(conn)
            busy = conn.execute("SELECT COUNT(*) FROM slots WHERE name=? AND occupied=1",
                                (name,)).fetchone()[0]
            if busy:
                return {"ok": False, "error": "这个配送点还有 %d 件货，先清空再删" % busy}
            conn.execute("DELETE FROM slots WHERE name=? AND kind='dst'", (name,))
            conn.commit()
        finally:
            conn.close()
        return {"ok": True, "removed": name}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.get("/api/slot/{slot_id}/parcels")
def api_slot_parcels(slot_id: str, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 某个货架格里的货物列表（点货架时用）
       货架格号存在 parcels.shelf_no 里（如 "1A"）"""
    try:
        # ★★ 只列【真在这个货架上】的货
        #    AT_GATE 在站点货架 / PLACED 在配送点货柜 / CREATED 刚揽收 ✓
        on_shelf = ("AT_GATE", "PLACED", "CREATED")
        rows = [p for p in db.list_parcels(limit=500)
                if str(p.get("shelf_no") or "").strip().upper() == slot_id.strip().upper()
                and str(p.get("status") or "").upper() in on_shelf]
        rows.sort(key=lambda r: r.get("created_at") or 0, reverse=True)
        return {"ok": True, "slot": slot_id, "count": len(rows), "parcels": rows}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/parcel/add")
async def api_parcel_add(request: Request, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 手动添加一件货（后台/管理员用）
       body: {tracking_no, order_no, owner, phone, dst, shelf_no, mode, pick_code}
    """
    try:
        body = await request.json()
    except Exception:
        body = {}
    try:
        # 自动生成包裹号 P0001 / P0002 ...
        existing = db.list_parcels(limit=2000)
        mx = 0
        for p in existing:
            pid = str(p.get("parcel_id") or "")
            if pid.startswith("P") and pid[1:].isdigit():
                mx = max(mx, int(pid[1:]))
        pid = "P%04d" % (mx + 1)
        import random as _r
        pick = str(body.get("pick_code") or "").strip() or "%04d" % _r.randint(1000, 9999)
        db.upsert_parcel(
            pid,
            tracking_no=str(body.get("tracking_no") or "").strip(),
            order_no=str(body.get("order_no") or "").strip(),
            owner=str(body.get("owner") or "").strip(),
            phone=str(body.get("phone") or "").strip(),
            dst=str(body.get("dst") or "").strip(),
            shelf_no=str(body.get("shelf_no") or "").strip().upper(),
            mode=str(body.get("mode") or "SHELF"),
            pick_code=pick,
            status=str(body.get("status") or "PLACED"),
        )
        db.log_event("平台", "parcel_add", {"parcel_id": pid}, parcel_id=pid)
        row = db.get_parcel(pid)
        # ★ 同步货架占用（让"货架状态"页看到）
        try:
            sn = str(body.get("shelf_no") or "").strip().upper()
            if sn:
                site = sn[0] if sn else "1"
                cur = db.list_slots()
                me = [x for x in cur if x.get("slot_id") == sn]
                n = len(db.list_parcels(limit=2000))  # 占位，下面重算
                cnt = sum(1 for p in db.list_parcels(limit=2000)
                          if str(p.get("shelf_no") or "").upper() == sn
                          and str(p.get("status") or "") not in ("PICKED",))
                db.upsert_slot(sn, site, cnt > 0, cnt)
        except Exception:
            pass
        return {"ok": True, "parcel": row}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/parcel/{parcel_id}/delete")
async def api_parcel_delete(parcel_id: str, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 删除一件货（连同它的事件记录）"""
    try:
        p = db.get_parcel(parcel_id)
        if not p:
            return {"ok": False, "error": "包裹不存在"}
        conn = db._connect()
        try:
            conn.execute("DELETE FROM events WHERE parcel_id=?", (parcel_id,))
            conn.execute("DELETE FROM parcels WHERE parcel_id=?", (parcel_id,))
            conn.commit()
        finally:
            conn.close()
        # ★ 重算该货架占用
        try:
            sn = str(p.get("shelf_no") or "").strip().upper()
            if sn:
                cnt = sum(1 for x in db.list_parcels(limit=2000)
                          if str(x.get("shelf_no") or "").upper() == sn
                          and str(x.get("status") or "") not in ("PICKED",))
                db.upsert_slot(sn, sn[0], cnt > 0, cnt)
        except Exception:
            pass
        return {"ok": True, "deleted": parcel_id}
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ==================================================================== ★★ 无人机端
# ------------------------------------------------------------------ ★ 用户收货点
@app.get("/api/my/dst")
def api_my_dst(phone: str = "", _u: Dict[str, Any] = Depends(_dep_user)):
    """★ 查我的收货点（手机号以登录态为准）"""
    ph = (phone or "").strip() if _u.get("role") == "admin" else (_u.get("phone") or "")
    return {"ok": True, "phone": ph, "dst_point": db.get_user_dst(ph)}


class DstIn(BaseModel):
    phone: str = ""
    dst_point: str = ""


@app.post("/api/my/dst")
def api_my_dst_set(b: DstIn, _u: Dict[str, Any] = Depends(_dep_user)):
    """★ 设置我的收货点（用户自己选送到哪）"""
    # ★★ 只能设自己的收货点（管理员可以代设）
    ph = (b.phone or "").strip() if _u.get("role") == "admin" else (_u.get("phone") or "")
    if not ph:
        return JSONResponse({"ok": False, "detail": "登录态里没有手机号，无法设置"})
    try:
        db.set_user_dst(ph, (b.dst_point or "").strip())
        return {"ok": True, "dst_point": b.dst_point}
    except Exception as e:
        return {"ok": False, "error": str(e)}


class SetDestIn(BaseModel):
    parcel_id: str
    phone: str = ""          # ★ 用于校验"只能改自己的"
    dst: str = ""


@app.post("/api/parcel/set_dest")
def api_parcel_set_dest(b: SetDestIn, _u: Dict[str, Any] = Depends(_dep_user)):
    """★★ 用户改【自己那件】包裹的送到哪

    安全：只允许改 phone 与包裹收件人一致的包裹（防越权）
    """
    try:
        p = db.get_parcel(b.parcel_id)
        if not p:
            return JSONResponse({"ok": False, "detail": "包裹不存在"})
        # ★★★ 目的地【装车即锁定】：小车一取走货，就不能再改目的地了
        #     否则小车正跑着被改目的地 → 得重新规划路线，不合理 ✗
        #     允许修改的只有【小车还没取走】的阶段：已揽收 / 无人机运输中 / 已到校门口站点
        st = str(p.get("status") or "").upper()
        if st not in ("CREATED", "ON_DRONE", "AT_GATE"):
            _why = {"LOADED": "已被小车装载", "DELIVERING": "小车正在配送途中",
                    "PLACED": "已放入货柜", "PICKED": "已被取件"}.get(st, st)
            return JSONResponse({"ok": False,
                                 "detail": "目的地已锁定（%s），不能再改。"
                                           "要改请在小车取货前设置。" % _why})
        # ★★★ 越权检查：以【登录态的手机号】为准（不再信前端传的 b.phone）
        p_phone = str(p.get("phone") or "").strip()
        me_phone = (_u.get("phone") or "").strip()
        if _u.get("role") != "admin":
            if p_phone and me_phone and me_phone != p_phone:
                return JSONResponse({"ok": False, "detail": "只能修改自己的包裹"})
        db.update_parcel(b.parcel_id, dst=(b.dst or "").strip())
        try:
            db.log_event(me_phone or _u.get("name") or "user", "set_dest",
                         {"parcel_id": b.parcel_id, "dst": b.dst},
                         parcel_id=b.parcel_id)
        except Exception:
            pass
        return {"ok": True, "parcel_id": b.parcel_id, "dst": b.dst}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.get("/api/drones")
def api_drones(_u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 无人机列表（页面上的"选择无人机"下拉用）

    ★★ 在线判定必须和 /api/devices_full 一致：按【多久没上报】算，
       不能只看数据库里的 status 字段（那是上次写入的，会一直是 idle）。
       >60 秒没上报（DEV_OFFLINE_AFTER）就算离线。
    """
    try:
        now = int(time.time())
        ds = db.list_drones()
        if not ds:
            # 库里还没有任何无人机 → 给个默认，页面不至于空着
            ds = [{"device_id": "nx-01", "role": "drone", "status": "offline",
                   "name": "", "last_seen": 0, "online": False}]
        for d in ds:
            ls = int(d.get("last_seen") or 0)
            d["online"] = bool(ls) and ((now - ls) <= DEV_OFFLINE_AFTER)
            # ★ 离线 → 状态一律显示 offline（数据库里的 idle 是陈旧的，会误导）
            d["status"] = "offline" if not d["online"] else (d.get("status") or "idle")
            d["age_sec"] = (now - ls) if ls else None
        return {"ok": True, "drones": ds}
    except Exception as e:
        return {"ok": False, "error": str(e)}


class DevIn(BaseModel):
    device_id: str
    role: str = "drone"
    name: str = ""


@app.post("/api/device/register")
def api_device_register(b: DevIn, _u: Dict[str, Any] = Depends(_dep_admin)):
    """★ 登记一台设备（无人机还没上报过也能先出现在下拉里）"""
    did = (b.device_id or "").strip()
    if not did:
        return JSONResponse({"ok": False, "detail": "设备编号必填"})
    try:
        db.register_device(did, b.role or "drone", (b.name or "").strip())
        return {"ok": True, "device_id": did}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/device/remove")
def api_device_remove(b: DevIn, _u: Dict[str, Any] = Depends(_dep_admin)):
    try:
        db.delete_device((b.device_id or "").strip())
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/gps")
async def api_gps(request: Request, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 无人机 GPS 位置上报
       body: {device, lat, lon, alt, status?}
       也接受 GET 风格 query：/api/gps?device=nx-01&lat=..&lon=.."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    try:
        dev = str(body.get("device") or "nx-01")
        lat = float(body.get("lat"))
        lon = float(body.get("lon"))
        alt = float(body.get("alt") or 0)
        st = str(body.get("status") or "")
    except Exception:
        return {"ok": False, "error": "参数不对：需要 device / lat / lon"}
    try:
        db.update_device_gps(dev, lat, lon, alt, role="drone", status=st)
        db.add_track(dev, lat=lat, lon=lon, alt=alt)
        return {"ok": True, "device": dev, "lat": lat, "lon": lon}
    except Exception as e:
        return {"ok": False, "error": str(e)}


class DroneBtn(BaseModel):
    """★ 无人机端按钮（人在网页上点）
       action: takeoff(起飞) / loaded(已取货) / dropped(已放货到校门口)
    """
    action: str = "takeoff"
    device: str = "nx-01"
    site: str = "1"
    slot: str = ""
    count: int = 1
    tracking_no: str = ""
    note: str = ""
    owner: str = ""        # ★ 收件人
    phone: str = ""        # ★ 收件人手机
    dst: str = ""          # ★ 送到哪（楼栋/驿站）
    marker_id: str = ""    # ★★ AR 码序号（货物身份证；不填平台自动分配）


@app.post("/api/drone/action")
def api_drone_action(b: DroneBtn, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★★★ 无人机端：人点按钮 → 平台记事件（/ 派车）
       · takeoff  → 记录"无人机已起飞"（+ 起飞时间）
       · loaded   → 记录"已取货"（挂载了哪些货）
       · dropped  → 记录"已放货到校门口站点" → ★ 走原有 drop_done 逻辑 → 生成任务派车
    """
    act = (b.action or "").strip().lower()
    ev = {"takeoff": "drone_takeoff", "loaded": "drone_loaded",
          "dropped": "drop_done"}.get(act)
    if not ev:
        return JSONResponse({"ok": False, "detail": "未知动作：%s（可用 takeoff/loaded/dropped）" % act})
    # ★★★ 离线保护：无人机不在线（机载程序没在跑）→ 不允许在网页替它上报
    did = (b.device or "nx-01").strip()
    if REQUIRE_ONLINE and not _is_online(did):
        return JSONResponse({"ok": False, "detail": _offline_msg(did)})

    extra: Dict[str, Any] = {"site": b.site, "count": b.count}
    if b.slot:
        extra["slot"] = b.slot
    if b.tracking_no:
        extra["tracking_no"] = b.tracking_no
    if b.note:
        extra["note"] = b.note
    if b.owner:
        extra["owner"] = b.owner
    if b.phone:
        extra["phone"] = b.phone
    if b.dst:
        extra["dst"] = b.dst
    if b.marker_id not in (None, ""):
        extra["marker_id"] = b.marker_id
    # ★★ 以【无人机身份上报】给平台（hub.py）→ 复用已有的 drop_done 派车逻辑
    #    （无人机是人操作的：人点按钮 = 替无人机发一条上报，走和真机一样的通道）
    r = mqtt_cmd.report_as_device(b.device, "drone", ev, **extra)
    if r.get("ok"):
        db.log_event("web", "drone_" + act, {"device": b.device, **extra})
    return JSONResponse(r)


@app.get("/api/track/{device_id}")
def api_track(device_id: str, limit: int = 800, _u: Dict[str, Any] = Depends(_dep_staff)):
    """★ 某设备的轨迹点（地图画线用）"""
    try:
        pts = db.get_track(device_id, limit=limit)
        return {"ok": True, "device": device_id, "count": len(pts), "points": pts}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.post("/api/track/{device_id}/clear")
def api_track_clear(device_id: str, _u: Dict[str, Any] = Depends(_dep_admin)):
    try:
        db.clear_track(device_id)
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ==================================================================== ★★ 角色
class RoleIn(BaseModel):
    user_id: str
    role: str = "user"          # admin / drone_admin / car_admin / user


@app.post("/api/set_role")
def api_set_role(b: RoleIn, _u: Dict[str, Any] = Depends(_dep_admin)):
    """★ 改用户角色（只有超级管理员能做；前端限制 + 这里也校验一下）"""
    if b.role not in ("admin", "drone_admin", "car_admin", "user"):
        return JSONResponse({"ok": False, "detail": "角色只能是 admin/drone_admin/car_admin/user"})
    try:
        db.upsert_user(user_id=b.user_id, role=b.role)
        return {"ok": True, "user_id": b.user_id, "role": b.role}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@app.get("/api/map")
def api_map():
    """★ 地图元数据（处理后的分辨率/原点/尺寸）"""
    if not os.path.isfile(os.path.join(MAP_DIR, "map.yaml")):
        return JSONResponse({"ok": False, "detail": "还没上传地图（把 map.pgm / map.yaml 放到 web平台/data/）"})
    try:
        _, meta = _prepare_map()
        if meta is None:
            return JSONResponse({"ok": False, "detail": "地图文件不完整（需要 map.pgm 和 map.yaml）"})
        return {"ok": True, **meta}
    except Exception as e:
        return JSONResponse({"ok": False, "detail": str(e)})


@app.get("/api/map.png")
def api_map_png():
    """★ 处理后的地图 PNG（已裁剪 + 放大，浏览器直接当背景图）"""
    try:
        png, meta = _prepare_map()
    except Exception as e:
        return JSONResponse({"ok": False, "detail": str(e)}, status_code=501)
    if png is None:
        return JSONResponse({"ok": False, "detail": "没有地图文件"}, status_code=404)
    return Response(content=png, media_type="image/png",
                    headers={"Cache-Control": "no-cache"})


@app.get("/api/health")
def api_health():
    return {"ok": True, "broker": "%s:%s" % (mqtt_cmd.BROKER, mqtt_cmd.PORT),
            "has_cred": bool(mqtt_cmd.USERNAME and mqtt_cmd.PASSWORD)}


if __name__ == "__main__":
    import uvicorn
    db.init_db()
    # ★ 端口可用环境变量 WEB_PORT 覆盖（默认 8010：服务器上 8000/8001 已被别人的项目占用）
    PORT = int(os.environ.get("WEB_PORT", "8010"))
    print("=" * 58)
    print("  车-机协同末端配送 · 监控台")
    print("  数据库:", os.path.abspath(db.DB_PATH))
    print("  MQTT:  %s:%s" % (mqtt_cmd.BROKER, mqtt_cmd.PORT))
    print("  浏览器访问:  http://<服务器IP>:%d" % PORT)
    print("  接口文档:    http://<服务器IP>:%d/docs" % PORT)
    print("=" * 58)
    uvicorn.run(app, host="0.0.0.0", port=PORT)
