# -*- coding: utf-8 -*-
"""
数据库访问层（SQLite）
======================
设计要点：
  ① 只用标准库 sqlite3 —— 零安装、无需服务 ✓
  ② 开 WAL 模式：读写可并行，多进程访问更稳 ✓
  ③ ★ 写入者只有 hub.py（本文件被它和 FastAPI 共用）
     网上说的"SQLite 并发写"问题，在本架构里不存在：
     小车/无人机不直接写库（它们走 MQTT），数据库只在服务器本机

数据库文件位置：环境变量 DELIVERY_DB，默认 <项目>/data/delivery.db
Python 3.8 兼容（不用 3.9+ 语法）
"""
import os
import json
import sqlite3
import time
from typing import Any, Dict, List, Optional

_HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DB = os.path.join(_HERE, "..", "data", "delivery.db")
DB_PATH = os.environ.get("DELIVERY_DB") or DEFAULT_DB
SCHEMA = os.path.join(_HERE, "schema.sql")


def _connect():
    """建立连接：开启 WAL + 外键 + 行工厂"""
    d = os.path.dirname(os.path.abspath(DB_PATH))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)
    conn = sqlite3.connect(DB_PATH, timeout=10)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")       # ★ 读写可并行
    conn.execute("PRAGMA synchronous=NORMAL")     # 性能与安全的平衡
    conn.execute("PRAGMA busy_timeout=5000")      # 忙时等 5 秒而不是直接报错
    return conn


def init_db():
    """建表（幂等，重复执行也安全）"""
    with open(SCHEMA, encoding="utf-8") as f:
        sql = f.read()
    conn = _connect()
    try:
        conn.executescript(sql)
        conn.commit()
    finally:
        conn.close()


def _now() -> int:
    return int(time.time())


# ---------------------------------------------------------------- 设备
def upsert_device(device_id: str, role: str = "", status: str = "idle"):
    conn = _connect()
    try:
        conn.execute(
            "INSERT INTO devices(device_id, role, status, last_seen) VALUES(?,?,?,?) "
            "ON CONFLICT(device_id) DO UPDATE SET role=excluded.role, "
            "status=excluded.status, last_seen=excluded.last_seen",
            (device_id, role, status, _now()))
        conn.commit()
    finally:
        conn.close()


def list_devices() -> List[Dict[str, Any]]:
    conn = _connect()
    try:
        rows = conn.execute("SELECT * FROM devices ORDER BY device_id").fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()


# ---------------------------------------------------------------- 库位
def upsert_slot(slot_id: str, site: str, occupied: bool,
                count: int = 0, task_id: Optional[str] = None):
    conn = _connect()
    try:
        conn.execute(
            "INSERT INTO slots(slot_id, site, occupied, count, task_id, updated_at) "
            "VALUES(?,?,?,?,?,?) "
            "ON CONFLICT(slot_id) DO UPDATE SET occupied=excluded.occupied, "
            "count=excluded.count, task_id=excluded.task_id, updated_at=excluded.updated_at",
            (slot_id, site, 1 if occupied else 0, count, task_id, _now()))
        conn.commit()
    finally:
        conn.close()


def list_slots() -> List[Dict[str, Any]]:
    conn = _connect()
    try:
        rows = conn.execute("SELECT * FROM slots ORDER BY slot_id").fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()


# ---------------------------------------------------------------- 任务
def insert_task(task_id: str, site: str, slot: str, count: int,
                status: str = "PENDING"):
    conn = _connect()
    try:
        t = _now()
        conn.execute(
            "INSERT OR REPLACE INTO tasks"
            "(task_id, site, slot, count, status, created_at, updated_at) "
            "VALUES(?,?,?,?,?,?,?)",
            (task_id, site, slot, count, status, t, t))
        conn.commit()
    finally:
        conn.close()


def upsert_task(task_id: str, site: str, slot: str, count: int, status: str,
                car_id: Optional[str] = None):
    """★ 幂等写入任务：不存在则新建（记 created_at），存在则只更新状态类字段"""
    conn = _connect()
    try:
        t = _now()
        row = conn.execute("SELECT task_id FROM tasks WHERE task_id=?", (task_id,)).fetchone()
        if row is None:
            conn.execute(
                "INSERT INTO tasks(task_id, site, slot, count, car_id, status,"
                " created_at, updated_at) VALUES(?,?,?,?,?,?,?,?)",
                (task_id, site, slot, count, car_id, status, t, t))
        else:
            conn.execute(
                "UPDATE tasks SET site=?, slot=?, count=?, car_id=?, status=?, updated_at=?"
                " WHERE task_id=?",
                (site, slot, count, car_id, status, t, task_id))
        conn.commit()
    finally:
        conn.close()


def update_task(task_id: str, **fields):
    """更新任务字段，例：update_task('T0001', status='DONE', delivered_at=...)"""
    if not fields:
        return
    allowed = {"site", "slot", "count", "car_id", "status",
               "created_at", "assigned_at", "picked_at", "delivered_at"}
    keys = [k for k in fields if k in allowed]
    if not keys:
        return
    fields["updated_at"] = _now()
    keys = keys + ["updated_at"]
    sets = ", ".join("%s=?" % k for k in keys)
    vals = [fields[k] for k in keys] + [task_id]
    conn = _connect()
    try:
        conn.execute("UPDATE tasks SET %s WHERE task_id=?" % sets, vals)
        conn.commit()
    finally:
        conn.close()


def list_tasks(limit: int = 100, site: Optional[str] = None,
               status: Optional[str] = None) -> List[Dict[str, Any]]:
    sql = "SELECT * FROM tasks WHERE 1=1"
    args: List[Any] = []
    if site:
        sql += " AND site=?"
        args.append(site)
    if status:
        sql += " AND status=?"
        args.append(status)
    sql += " ORDER BY created_at DESC LIMIT ?"
    args.append(int(limit))
    conn = _connect()
    try:
        return [dict(r) for r in conn.execute(sql, args).fetchall()]
    finally:
        conn.close()


# ---------------------------------------------------------------- 派单队列
def enqueue(site: str, task_id: str):
    conn = _connect()
    try:
        conn.execute("INSERT INTO queue(site, task_id, enqueue_at) VALUES(?,?,?)",
                     (site, task_id, _now()))
        conn.commit()
    finally:
        conn.close()


def dequeue_head(site: str) -> Optional[str]:
    """从队头取一个任务（先进先出），并把它从队列里删掉"""
    conn = _connect()
    try:
        row = conn.execute(
            "SELECT id, task_id FROM queue WHERE site=? ORDER BY id LIMIT 1",
            (site,)).fetchone()
        if not row:
            return None
        conn.execute("DELETE FROM queue WHERE id=?", (row["id"],))
        conn.commit()
        return row["task_id"]
    finally:
        conn.close()


def replace_queue(items):
    """整体重写队列（items = [(site, task_id), ...]）—— hub.py 用它同步内存队列"""
    conn = _connect()
    try:
        conn.execute("DELETE FROM queue")
        t = _now()
        for site, tid in items:
            conn.execute("INSERT INTO queue(site, task_id, enqueue_at) VALUES(?,?,?)",
                         (site, tid, t))
        conn.commit()
    finally:
        conn.close()


def list_queue(site: Optional[str] = None) -> List[Dict[str, Any]]:
    sql = "SELECT * FROM queue"
    args: List[Any] = []
    if site:
        sql += " WHERE site=?"
        args.append(site)
    sql += " ORDER BY id"
    conn = _connect()
    try:
        return [dict(r) for r in conn.execute(sql, args).fetchall()]
    finally:
        conn.close()


def queue_size(site: Optional[str] = None) -> int:
    conn = _connect()
    try:
        if site:
            r = conn.execute("SELECT COUNT(*) c FROM queue WHERE site=?", (site,)).fetchone()
        else:
            r = conn.execute("SELECT COUNT(*) c FROM queue").fetchone()
        return int(r["c"])
    finally:
        conn.close()


# ---------------------------------------------------------------- 事件日志
def log_event(device: str, event: str, payload: Any = None):
    if not isinstance(payload, str):
        try:
            payload = json.dumps(payload, ensure_ascii=False)
        except Exception:
            payload = str(payload)
    conn = _connect()
    try:
        conn.execute("INSERT INTO events(device, event, payload, ts) VALUES(?,?,?,?)",
                     (device, event, payload, _now()))
        conn.commit()
    finally:
        conn.close()


def list_events(limit: int = 100, device: Optional[str] = None) -> List[Dict[str, Any]]:
    sql = "SELECT * FROM events"
    args: List[Any] = []
    if device:
        sql += " WHERE device=?"
        args.append(device)
    sql += " ORDER BY id DESC LIMIT ?"
    args.append(int(limit))
    conn = _connect()
    try:
        return [dict(r) for r in conn.execute(sql, args).fetchall()]
    finally:
        conn.close()


# ---------------------------------------------------------------- ★ 统计
def stats() -> Dict[str, Any]:
    """给论文用的统计：完成数 / 平均耗时 / 成功率 / 库位占用"""
    conn = _connect()
    try:
        total = conn.execute("SELECT COUNT(*) c FROM tasks").fetchone()["c"]
        done = conn.execute("SELECT COUNT(*) c FROM tasks WHERE status='DONE'").fetchone()["c"]
        failed = conn.execute("SELECT COUNT(*) c FROM tasks WHERE status='FAILED'").fetchone()["c"]
        # ★ 平均耗时 = 送达时间 - 创建时间（只统计已完成的）
        rows = conn.execute(
            "SELECT created_at, delivered_at FROM tasks "
            "WHERE status='DONE' AND delivered_at IS NOT NULL AND created_at IS NOT NULL"
        ).fetchall()
        durs = [r["delivered_at"] - r["created_at"] for r in rows
                if r["delivered_at"] and r["created_at"] and r["delivered_at"] >= r["created_at"]]
        avg = round(sum(durs) / len(durs), 1) if durs else None
        occ = conn.execute("SELECT COUNT(*) c FROM slots WHERE occupied=1").fetchone()["c"]
        free = conn.execute("SELECT COUNT(*) c FROM slots WHERE occupied=0").fetchone()["c"]
        return {
            "tasks_total": total,
            "tasks_done": done,
            "tasks_failed": failed,
            "success_rate": round(done * 100.0 / total, 1) if total else None,
            "avg_duration_sec": avg,
            "avg_duration_sec_samples": len(durs),
            "slots_occupied": occ,
            "slots_free": free,
        }
    finally:
        conn.close()


if __name__ == "__main__":
    init_db()
    print("✅ 数据库已初始化:", os.path.abspath(DB_PATH))
    print("   统计:", stats())
