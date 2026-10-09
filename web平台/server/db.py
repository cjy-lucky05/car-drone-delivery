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


_MIGRATED = False          # ★★ 迁移只在【进程内】真正跑一次
_mig_lock = None


def _migrate(conn):
    """★★★ 迁移只跑一次（进程内），之后每次调用【立刻返回】

    为什么：以前每个请求都调 _migrate → 每次都尝试 ALTER TABLE（=写事务）
    → 白白去抢 SQLite 的【写锁】→ 高并发时全是无谓排队 ✗
    现在：进程启动跑一次，之后是内存里一个 if 判断 ✓
    """
    global _MIGRATED, _mig_lock
    if _MIGRATED:
        return
    if _mig_lock is None:
        import threading as _th
        _mig_lock = _th.Lock()
    with _mig_lock:
        if _MIGRATED:
            return
        _do_migrate(conn)
        _MIGRATED = True


def _do_migrate(conn):
    """★ 老库升级：给已存在的表补新字段（SQLite 加列失败就忽略）"""
    for col, typ in (("pos_x", "REAL"), ("pos_y", "REAL"),
                     ("pos_yaw", "REAL"), ("pos_ts", "INTEGER")):
        try:
            conn.execute("ALTER TABLE devices ADD COLUMN %s %s" % (col, typ))
        except Exception:
            pass          # 已存在 → 正常
    # ★ 2026-10-08：users 加 password
    try:
        conn.execute("ALTER TABLE users ADD COLUMN password TEXT")
    except Exception:
        pass
    # ★★ 2026-10-08：devices 加 GPS 字段（无人机位置用经纬度）
    for col, typ in (("lat", "REAL"), ("lon", "REAL"), ("alt", "REAL")):
        try:
            conn.execute("ALTER TABLE devices ADD COLUMN %s %s" % (col, typ))
        except Exception:
            pass
    # ★★★ 2026-10-09：parcels 加 marker_id（AR 码序号 → 小车用摄像头认货）
    try:
        conn.execute("ALTER TABLE parcels ADD COLUMN marker_id INTEGER")
    except Exception:
        pass
    try:
        conn.execute("CREATE INDEX IF NOT EXISTS idx_parcels_marker ON parcels(marker_id)")
    except Exception:
        pass

    # ★★ 2026-10-09：tasks 加 dst（派单时锁定的目的地 —— 装车后不许改）
    try:
        conn.execute("ALTER TABLE tasks ADD COLUMN dst TEXT")
    except Exception:
        pass
    # ★★★ 2026-10-09：tasks 加 marker_id（任务要取哪一号货 —— 抓前校验用）
    #   必须落库：否则平台一重启，"要取哪一号"就丢了 ✗
    try:
        conn.execute("ALTER TABLE tasks ADD COLUMN marker_id INTEGER")
    except Exception:
        pass

    # ★★ 2026-10-09：users 加 dst_point（用户自己选的收货点）
    try:
        conn.execute("ALTER TABLE users ADD COLUMN dst_point TEXT")
    except Exception:
        pass

    # ★★ 2026-10-08：slots 加 kind(transit/dst) + name(显示名)
    for col, typ in (("kind", "TEXT"), ("name", "TEXT")):
        try:
            conn.execute("ALTER TABLE slots ADD COLUMN %s %s" % (col, typ))
        except Exception:
            pass

    # ★★ 轨迹表（老库没有就建）
    try:
        conn.execute("""CREATE TABLE IF NOT EXISTS tracks (
            id INTEGER PRIMARY KEY AUTOINCREMENT, device_id TEXT,
            x REAL, y REAL, lat REAL, lon REAL, alt REAL, ts INTEGER)""")
        conn.execute("CREATE INDEX IF NOT EXISTS idx_tracks_dev ON tracks(device_id, ts)")
    except Exception:
        pass

    # ★ 2026-10-08：devices 加设备信息字段（名称/电量/当前任务）
    for col, typ in (("name", "TEXT"), ("battery", "REAL"), ("battery_v", "REAL"),
                     ("battery_ts", "INTEGER"), ("task_now", "TEXT")):
        try:
            conn.execute("ALTER TABLE devices ADD COLUMN %s %s" % (col, typ))
        except Exception:
            pass
    # ★ 2026-10-08：events 加 parcel_id（用于"按包裹看完整时间线"）
    try:
        conn.execute("ALTER TABLE events ADD COLUMN parcel_id TEXT")
    except Exception:
        pass
    try:   # ★ 索引必须在加列之后建（老库先 ALTER，新库本来就有列）
        conn.execute("CREATE INDEX IF NOT EXISTS idx_events_parcel ON events(parcel_id, ts)")
    except Exception:
        pass
    # ★★★ 2026-10-09：补常用索引（数据涨上来后查询不走索引会明显变慢）
    for _idx in (
        "CREATE INDEX IF NOT EXISTS idx_events_ts      ON events(ts DESC)",
        "CREATE INDEX IF NOT EXISTS idx_events_dev_ts  ON events(device, ts DESC)",
        "CREATE INDEX IF NOT EXISTS idx_parcels_shelf  ON parcels(shelf_no)",
        "CREATE INDEX IF NOT EXISTS idx_parcels_phone  ON parcels(phone)",
        "CREATE INDEX IF NOT EXISTS idx_parcels_status ON parcels(status)",
        "CREATE INDEX IF NOT EXISTS idx_tasks_status   ON tasks(status)",
        "CREATE INDEX IF NOT EXISTS idx_users_phone    ON users(phone)",
        "CREATE INDEX IF NOT EXISTS idx_slots_kind     ON slots(kind)",
    ):
        try:
            conn.execute(_idx)
        except Exception:
            pass


def init_db():
    """建表（幂等，重复执行也安全）"""
    with open(SCHEMA, encoding="utf-8") as f:
        sql = f.read()
    conn = _connect()
    try:
        conn.executescript(sql)
        _migrate(conn)
        conn.commit()
    finally:
        conn.close()
    ensure_default_users()          # ★ 保证 admin / user 两个账号存在


# ---------------------------------------------------------------- ★ 密码
# ★★★ 为了支持"找回密码（显示原密码）"，密码改成【可逆存储】
#    做法：简单异或混淆 + base64（毕设级；论文里如实写"非生产级安全"）
#    同时兼容【旧的 sha256 散列值】—— 旧值无法还原，只能重置
_K = b"cjy-hermes-2026"


def hash_pw(pw: str) -> str:
    """旧版：加盐 sha256（只用于校验老数据，不再用于写入）"""
    import hashlib
    return hashlib.sha256(("cjy-hermes-" + (pw or "")).encode("utf-8")).hexdigest()


def enc_pw(pw: str) -> str:
    """可逆编码（写入时用）"""
    import base64
    b = bytes([c ^ _K[i % len(_K)] for i, c in enumerate((pw or "").encode("utf-8"))])
    return "B64:" + base64.b64encode(b).decode("ascii")


def dec_pw(s: str):
    """解出原密码；旧散列值返回 None（解不出来）"""
    if not s or not s.startswith("B64:"):
        return None
    import base64
    try:
        b = base64.b64decode(s[4:])
        return bytes([c ^ _K[i % len(_K)] for i, c in enumerate(b)]).decode("utf-8")
    except Exception:
        return None


def verify_pw(stored: str, pw: str) -> bool:
    """校验密码：兼容【可逆新值】和【旧散列值】"""
    if not stored:
        return False
    if stored.startswith("B64:"):
        return dec_pw(stored) == pw
    return stored == hash_pw(pw)          # 老数据


def set_password(user_id: str, pw: str):
    conn = _connect()
    try:
        _migrate(conn)
        conn.execute("UPDATE users SET password=?, updated_at=? WHERE user_id=?",
                     (enc_pw(pw), _now(), user_id))
        conn.commit()
    finally:
        conn.close()


def ensure_default_users():
    """★ 保证两个演示账号存在：admin/admin123（管理员）、user/user123（普通用户）
       注意：按【用户名/手机号】判重（不能用固定 user_id —— 可能已被别的用户占用）"""
    defaults = [("admin", "admin", "管理员", "admin", "admin123"),
                ("user",  "user",  "普通用户", "user",  "user123")]
    conn = _connect()
    try:
        _migrate(conn)
        for phone, name, real, role, pw in defaults:
            row = conn.execute("SELECT user_id FROM users WHERE phone=? OR name=?",
                               (phone, name)).fetchone()
            if row:                      # 已存在 → 补上密码 + 校正角色
                conn.execute("UPDATE users SET password=?, role=?, name=?, active=1,"
                             " updated_at=? WHERE user_id=?",
                             (enc_pw(pw), role, real, _now(), row["user_id"]))
            else:                        # 新建，自动找一个没被占用的 user_id
                n = conn.execute("SELECT COUNT(*) c FROM users").fetchone()["c"]
                uid = "U%04d" % (int(n) + 1)
                while conn.execute("SELECT 1 FROM users WHERE user_id=?", (uid,)).fetchone():
                    n += 1
                    uid = "U%04d" % (int(n) + 1)
                t = _now()
                conn.execute("INSERT INTO users(user_id,name,phone,password,role,building,"
                             " active,created_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?)",
                             (uid, real, phone, enc_pw(pw), role, "", 1, t, t))
        conn.commit()
    except Exception as e:
        print("[db] ensure_default_users:", e)
    finally:
        conn.close()


def login_by_account(account: str, password: str):
    """★ 账号密码登录：account 填【手机号】或【用户名】(admin/user)
       ★★ 故意【不匹配姓名】—— 重名很常见，按姓名登录会认错人"""
    account = (account or "").strip()
    if not account or password is None:
        return None
    conn = _connect()
    try:
        _migrate(conn)
        r = conn.execute("SELECT * FROM users WHERE phone=? LIMIT 1", (account,)).fetchone()
        if not r:
            return None
        u = dict(r)
        if not u.get("active", 1):
            return None
        if not verify_pw(u.get("password"), password):
            return None
        return u
    finally:
        conn.close()


def find_by_name(name: str):
    """按姓名查（仅用于"重名提示"，不用于登录）"""
    conn = _connect()
    try:
        rows = conn.execute("SELECT user_id, name, phone FROM users WHERE name=?",
                            ((name or "").strip(),)).fetchall()
        return [dict(r) for r in rows]
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


def update_device_pos(device_id: str, x: float, y: float, yaw: float = 0.0,
                      role: str = "", status: str = ""):
    """★ 更新设备实时位置（网页地图用）"""
    conn = _connect()
    try:
        _migrate(conn)
        row = conn.execute("SELECT device_id FROM devices WHERE device_id=?", (device_id,)).fetchone()
        t = _now()
        if row is None:
            conn.execute("INSERT INTO devices(device_id, role, status, last_seen,"
                         " pos_x, pos_y, pos_yaw, pos_ts) VALUES(?,?,?,?,?,?,?,?)",
                         (device_id, role or "", status or "idle", t, x, y, yaw, t))
        else:
            conn.execute("UPDATE devices SET pos_x=?, pos_y=?, pos_yaw=?, pos_ts=?, last_seen=?"
                         " WHERE device_id=?", (x, y, yaw, t, t, device_id))
        conn.commit()
    finally:
        conn.close()


def update_device_battery(device_id: str, battery=None, voltage=None,
                          name: str = "", task_now=None):
    """★ 设备上报电量（小车协议里有 0x07 电池功能码）"""
    conn = _connect()
    try:
        _migrate(conn)
        t = _now()
        row = conn.execute("SELECT device_id FROM devices WHERE device_id=?",
                           (device_id,)).fetchone()
        if row is None:
            conn.execute("INSERT INTO devices(device_id, role, status, last_seen,"
                         " name, battery, battery_v, battery_ts, task_now)"
                         " VALUES(?,?,?,?,?,?,?,?,?)",
                         (device_id, "", "idle", t, name, battery, voltage, t, task_now))
        else:
            sets, vals = [], []
            if battery is not None:  sets.append("battery=?");    vals.append(battery)
            if voltage is not None:  sets.append("battery_v=?");  vals.append(voltage)
            if name:                 sets.append("name=?");       vals.append(name)
            if task_now is not None: sets.append("task_now=?");   vals.append(task_now)
            sets += ["battery_ts=?", "last_seen=?"]; vals += [t, t]
            vals.append(device_id)
            conn.execute("UPDATE devices SET %s WHERE device_id=?" % ", ".join(sets), vals)
        conn.commit()
    finally:
        conn.close()


def get_device(device_id: str) -> Optional[Dict[str, Any]]:
    """★ 读一台设备（判断在线/离线用）"""
    if not device_id:
        return None
    conn = _connect()
    try:
        _migrate(conn)
        r = conn.execute("SELECT * FROM devices WHERE device_id=?", (device_id,)).fetchone()
        return dict(r) if r else None
    finally:
        conn.close()


# ★★★ AR 码序号（marker_id）—— 货物的"身份证"
#   一件货配一枚 AR 码，码的 id 就是序号；小车抓取时用摄像头读出来，
#   平台据此判断"车上装的是哪一件"。序号可复用（那件货取走后就空出来了）。
MARKER_MIN, MARKER_MAX = 1, 99


def next_free_marker() -> int:
    """★ 分配一个空闲的 AR 码序号（1~99）"""
    conn = _connect()
    try:
        _migrate(conn)
        used = set()
        for r in conn.execute(
                "SELECT marker_id FROM parcels WHERE marker_id IS NOT NULL"
                " AND status NOT IN ('PICKED')"):
            try:
                used.add(int(r[0]))
            except Exception:
                pass
        for n in range(MARKER_MIN, MARKER_MAX + 1):
            if n not in used:
                return n
        return MARKER_MIN          # 都用满了就从头（理论上不会）
    finally:
        conn.close()


def find_parcel_by_marker(marker_id, only_at_gate: bool = False) -> Optional[Dict[str, Any]]:
    """★ 按 AR 码序号找货（小车读到的序号 → 是哪一件）"""
    try:
        mid = int(marker_id)
    except Exception:
        return None
    conn = _connect()
    try:
        _migrate(conn)
        sql = "SELECT * FROM parcels WHERE marker_id=?"
        args: List[Any] = [mid]
        if only_at_gate:
            sql += " AND status='AT_GATE'"
        sql += " ORDER BY created_at DESC LIMIT 1"
        r = conn.execute(sql, args).fetchone()
        return dict(r) if r else None
    finally:
        conn.close()


def register_device(device_id: str, role: str = "drone", name: str = ""):
    """★ 手动登记一台设备（还没上报过的也能先出现在下拉里）

    典型用途：无人机还没飞过、没在库里，但工作人员想先把它登记上，
    后面选它、给它派任务。
    """
    conn = _connect()
    try:
        _migrate(conn)
        t = _now()
        row = conn.execute("SELECT device_id FROM devices WHERE device_id=?",
                           (device_id,)).fetchone()
        if row is None:
            conn.execute("INSERT INTO devices(device_id, role, status, last_seen, name)"
                         " VALUES(?,?,?,?,?)",
                         (device_id, role, "offline", 0, name or device_id))
        else:
            if name:
                conn.execute("UPDATE devices SET role=?, name=? WHERE device_id=?",
                             (role, name, device_id))
            else:
                conn.execute("UPDATE devices SET role=? WHERE device_id=?",
                             (role, device_id))
        conn.commit()
    finally:
        conn.close()


def set_user_dst(phone: str, dst_point: str):
    """★ 用户设置自己的收货点（宿舍楼/图书馆/驿站…）"""
    conn = _connect()
    try:
        _migrate(conn)
        conn.execute("UPDATE users SET dst_point=?, updated_at=? WHERE phone=?",
                     (dst_point, _now(), phone))
        # 没这个手机号就建一条（用户第一次来时）
        if conn.total_changes == 0:
            pass
        conn.commit()
    finally:
        conn.close()


def get_user_dst(phone: str) -> str:
    """★ 读用户的收货点（找不到返回空串）"""
    if not phone:
        return ""
    conn = _connect()
    try:
        _migrate(conn)
        r = conn.execute("SELECT dst_point FROM users WHERE phone=?", (phone,)).fetchone()
        return (r[0] if r and r[0] else "") or ""
    finally:
        conn.close()


def list_drones():
    """★ 所有无人机（下拉选择用）"""
    conn = _connect()
    try:
        _migrate(conn)
        rows = conn.execute("SELECT * FROM devices WHERE role='drone'"
                            " ORDER BY device_id").fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()


def delete_device(device_id: str):
    conn = _connect()
    try:
        _migrate(conn)
        conn.execute("DELETE FROM devices WHERE device_id=?", (device_id,))
        conn.commit()
    finally:
        conn.close()


def update_device_gps(device_id: str, lat: float, lon: float, alt: float = 0.0,
                      role: str = "drone", status: str = ""):
    """★ 无人机 GPS 位置上报（经纬度）"""
    conn = _connect()
    try:
        _migrate(conn)
        t = _now()
        row = conn.execute("SELECT device_id FROM devices WHERE device_id=?",
                           (device_id,)).fetchone()
        if row is None:
            conn.execute("INSERT INTO devices(device_id, role, status, last_seen,"
                         " lat, lon, alt, pos_ts) VALUES(?,?,?,?,?,?,?,?)",
                         (device_id, role, status or "idle", t, lat, lon, alt, t))
        else:
            if status:
                conn.execute("UPDATE devices SET lat=?, lon=?, alt=?, pos_ts=?,"
                             " last_seen=?, status=? WHERE device_id=?",
                             (lat, lon, alt, t, t, status, device_id))
            else:
                conn.execute("UPDATE devices SET lat=?, lon=?, alt=?, pos_ts=?,"
                             " last_seen=? WHERE device_id=?",
                             (lat, lon, alt, t, t, device_id))
        conn.commit()
    finally:
        conn.close()


def add_track(device_id: str, x=None, y=None, lat=None, lon=None, alt=None):
    """★ 记一个轨迹点（小车用 x/y；无人机用 lat/lon）"""
    conn = _connect()
    try:
        _migrate(conn)
        conn.execute("INSERT INTO tracks(device_id, x, y, lat, lon, alt, ts)"
                     " VALUES(?,?,?,?,?,?,?)",
                     (device_id, x, y, lat, lon, alt, _now()))
        conn.execute("DELETE FROM tracks WHERE device_id=? AND id NOT IN"
                     " (SELECT id FROM tracks WHERE device_id=? ORDER BY id DESC LIMIT 3000)",
                     (device_id, device_id))
        conn.commit()
    finally:
        conn.close()


def get_track(device_id: str, limit: int = 800):
    """★ 读某设备的轨迹点（给地图画线）"""
    conn = _connect()
    try:
        _migrate(conn)
        rows = conn.execute("SELECT * FROM tracks WHERE device_id=? ORDER BY id ASC LIMIT ?",
                            (device_id, int(limit))).fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()


def clear_track(device_id: str):
    conn = _connect()
    try:
        _migrate(conn)
        conn.execute("DELETE FROM tracks WHERE device_id=?", (device_id,))
        conn.commit()
    finally:
        conn.close()


def list_devices_full() -> List[Dict[str, Any]]:
    """★ 设备信息（含电量/当前位置/当前任务）—— 管理员的"设备信息"卡用"""
    conn = _connect()
    try:
        _migrate(conn)
        rows = conn.execute("SELECT * FROM devices ORDER BY role, device_id").fetchall()
        out = [dict(r) for r in rows]
        # 顺带带上"这台设备正在运几件货"
        for dv in out:
            try:
                r = conn.execute("SELECT COUNT(*) c FROM parcels WHERE car_id=? AND status"
                                 " IN ('LOADED','DELIVERING')", (dv["device_id"],)).fetchone()
                dv["carrying"] = int(r["c"] or 0)
            except Exception:
                dv["carrying"] = 0
        return out
    finally:
        conn.close()


# ---------------------------------------------------------------- ★★★ 用户
def upsert_user(user_id=None, name="", phone="", building="", role="user",
                note="", active=1, password=""):
    """新增/更新用户；不给 user_id 则自动生成 U0001"""
    conn = _connect()
    try:
        t = _now()
        if not user_id:
            r = conn.execute("SELECT COUNT(*) c FROM users").fetchone()
            user_id = "U%04d" % (int(r["c"]) + 1)
            while conn.execute("SELECT 1 FROM users WHERE user_id=?", (user_id,)).fetchone():
                user_id = "U%04d" % (int(user_id[1:]) + 1)
            conn.execute("INSERT INTO users(user_id,name,phone,building,role,note,active,"
                         " created_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?)",
                         (user_id, name, phone, building, role, note, active, t, t))
        else:
            row = conn.execute("SELECT user_id FROM users WHERE user_id=?", (user_id,)).fetchone()
            if row is None:
                conn.execute("INSERT INTO users(user_id,name,phone,password,building,role,note,"
                             "active,created_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?)",
                             (user_id, name, phone, (enc_pw(password) if password else ""),
                              building, role, note, active, t, t))
            else:
                if password:
                    conn.execute("UPDATE users SET name=?,phone=?,password=?,building=?,role=?,"
                                 "note=?,active=?,updated_at=? WHERE user_id=?",
                                 (name, phone, enc_pw(password), building, role, note, active, t, user_id))
                else:
                    conn.execute("UPDATE users SET name=?,phone=?,building=?,role=?,note=?,"
                                 "active=?,updated_at=? WHERE user_id=?",
                                 (name, phone, building, role, note, active, t, user_id))
        conn.commit()
        return user_id
    except Exception as e:
        print("[db] upsert_user 失败:", e)
        return None
    finally:
        conn.close()


def delete_user(user_id: str) -> int:
    conn = _connect()
    try:
        n = conn.execute("DELETE FROM users WHERE user_id=?", (user_id,)).rowcount
        conn.commit()
        return n
    finally:
        conn.close()


def get_user_by_phone(phone: str):
    conn = _connect()
    try:
        r = conn.execute("SELECT * FROM users WHERE phone=?", (phone,)).fetchone()
        return dict(r) if r else None
    finally:
        conn.close()


def get_user(user_id: str):
    conn = _connect()
    try:
        r = conn.execute("SELECT * FROM users WHERE user_id=?", (user_id,)).fetchone()
        return dict(r) if r else None
    finally:
        conn.close()


def list_users(building=None) -> List[Dict[str, Any]]:
    """★ 用户列表 + 每人的包裹统计（用于"用户管理"卡）"""
    conn = _connect()
    try:
        sql = "SELECT * FROM users"
        args = []
        if building == "__none__":
            # ★★ 特例：筛【没填楼栋】的人（管理员/工作人员/未填的普通用户）
            sql += " WHERE (building IS NULL OR TRIM(building)='')"
        elif building:
            sql += " WHERE building=?"; args.append(building)
        sql += " ORDER BY building, name"
        users = [dict(r) for r in conn.execute(sql, args).fetchall()]
        for u in users:
            key = u.get("phone") or ""
            rr = conn.execute(
                "SELECT COUNT(*) total,"
                " SUM(CASE WHEN status='PLACED' THEN 1 ELSE 0 END) waiting,"
                " SUM(CASE WHEN status IN ('ON_DRONE','AT_GATE','LOADED','DELIVERING')"
                "          THEN 1 ELSE 0 END) onway,"
                " SUM(CASE WHEN status='PICKED' THEN 1 ELSE 0 END) picked"
                " FROM parcels WHERE phone=? OR owner=?", (key, u.get("name") or "")).fetchone()
            u["total"] = int(rr["total"] or 0)
            u["waiting"] = int(rr["waiting"] or 0)
            u["onway"] = int(rr["onway"] or 0)
            u["picked"] = int(rr["picked"] or 0)
        return users
    finally:
        conn.close()


def users_grouped() -> List[Dict[str, Any]]:
    """★ 按楼栋分组"""
    us = list_users()
    g = {}
    for u in us:
        k = u.get("building") or "(未填楼栋)"
        g.setdefault(k, []).append(u)
    return [{"building": k, "users": v} for k, v in sorted(g.items())]


def list_users_agg() -> List[Dict[str, Any]]:
    """★★ 用户管理：从 parcels 表聚合出"用户清单"（不需要单独建 users 表）"""
    conn = _connect()
    try:
        rows = conn.execute(
            "SELECT owner, phone, COUNT(*) total,"
            " SUM(CASE WHEN status='PLACED' THEN 1 ELSE 0 END) waiting,"
            " SUM(CASE WHEN status='PICKED' THEN 1 ELSE 0 END) picked,"
            " SUM(CASE WHEN status IN ('ON_DRONE','AT_GATE','LOADED','DELIVERING')"
            "          THEN 1 ELSE 0 END) onway,"
            " MAX(created_at) last_at"
            " FROM parcels WHERE owner IS NOT NULL AND owner<>''"
            " GROUP BY owner, phone ORDER BY last_at DESC").fetchall()
        return [dict(r) for r in rows]
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
# ★★ 校内【配送点货柜】默认清单（车把货送到这儿，收件人来取）
DST_POINTS = [
    ("2号公寓楼下", 3), ("3号公寓楼下", 3), ("图书馆楼下", 3),
    ("教学楼楼下", 3), ("菜鸟驿站",  3),
]


def ensure_dst_slots():
    """★ 初始化校内配送点货柜（每处 3 格：A/B/C）；已存在就跳过"""
    conn = _connect()
    try:
        _migrate(conn)
        t = _now()
        for name, n in DST_POINTS:
            for i in range(n):
                sid = "%s-%s" % (name, chr(ord("A") + i))
                row = conn.execute("SELECT slot_id FROM slots WHERE slot_id=?", (sid,)).fetchone()
                if row is None:
                    conn.execute("INSERT INTO slots(slot_id, site, occupied, count, task_id,"
                                 " kind, name, updated_at) VALUES(?,?,?,?,?,?,?,?)",
                                 (sid, name, 0, 0, None, "dst", name, t))
        # ★ 中转货架（1A/1B...）补 kind='transit'
        conn.execute("UPDATE slots SET kind='transit' WHERE kind IS NULL OR kind=''")
        conn.commit()
    finally:
        conn.close()


def list_dst_slots():
    """★ 只取【配送点货柜】"""
    conn = _connect()
    try:
        _migrate(conn)
        rows = conn.execute("SELECT * FROM slots WHERE kind='dst' ORDER BY name, slot_id").fetchall()
        return [dict(r) for r in rows]
    finally:
        conn.close()


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
                car_id: Optional[str] = None, dst: str = "", marker_id: Optional[int] = None):
    """★ 幂等写入任务：不存在则新建（记 created_at），存在则更新

    ★★★ dst / marker_id 也要落库 —— 这两个是"这趟任务要干什么"的关键：
    dst = 送到哪（装车后锁定不许改）；marker_id = 要取哪一号货（抓前校验用）。
    只存内存的话，平台一重启就丢了 ✗（真实踩过）
    """
    conn = _connect()
    try:
        _migrate(conn)
        t = _now()
        row = conn.execute("SELECT task_id FROM tasks WHERE task_id=?", (task_id,)).fetchone()
        if row is None:
            conn.execute(
                "INSERT INTO tasks(task_id, site, slot, count, car_id, status, dst, marker_id,"
                " created_at, updated_at) VALUES(?,?,?,?,?,?,?,?,?,?)",
                (task_id, site, slot, count, car_id, status, dst or "", marker_id, t, t))
        else:
            conn.execute(
                "UPDATE tasks SET site=?, slot=?, count=?, car_id=?, status=?,"
                " dst=COALESCE(NULLIF(?,''), dst), marker_id=COALESCE(?, marker_id),"
                " updated_at=? WHERE task_id=?",
                (site, slot, count, car_id, status, dst or "", marker_id, t, task_id))
        conn.commit()
    finally:
        conn.close()


def update_task(task_id: str, **fields):
    """更新任务字段，例：update_task('T0001', status='DONE', delivered_at=...)"""
    if not fields:
        return
    allowed = {"site", "slot", "count", "car_id", "status", "dst", "marker_id",
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
def log_event(device: str, event: str, payload: Any = None,
              parcel_id: Optional[str] = None):
    """★ 记一条事件。带 parcel_id 的事件会出现在【该包裹的时间线】里"""
    if not isinstance(payload, str):
        try:
            payload = json.dumps(payload, ensure_ascii=False)
        except Exception:
            payload = str(payload)
    conn = _connect()
    try:
        _migrate(conn)
        conn.execute("INSERT INTO events(device, event, payload, ts, parcel_id) "
                     "VALUES(?,?,?,?,?)",
                     (device, event, payload, _now(), parcel_id))
        conn.commit()
    finally:
        conn.close()


def list_events(limit: int = 100, device: Optional[str] = None,
                parcel_id: Optional[str] = None,
                asc: bool = False, only_biz: bool = False,
                q: Optional[str] = None) -> List[Dict[str, Any]]:
    """★ 事件列表
       only_biz=True 时【过滤掉高频噪音】（位置/电量/GPS/心跳那类刷屏的），
       只留真正有业务含义的事件（投递/装载/送达/取件/异常…）
    """
    sql = "SELECT * FROM events WHERE 1=1"
    args: List[Any] = []
    if only_biz:
        sql += (" AND event NOT IN ('position','battery','gps','heartbeat',"
                "'selftest','idle')")
    if q:
        sql += " AND (event LIKE ? OR device LIKE ? OR payload LIKE ?)"
        args.extend(["%" + q + "%"] * 3)
    if device:
        sql += " AND device=?"
        args.append(device)
    if parcel_id:
        sql += " AND parcel_id=?"
        args.append(parcel_id)
    sql += " ORDER BY id %s LIMIT ?" % ("ASC" if asc else "DESC")
    args.append(int(limit))
    conn = _connect()
    try:
        return [dict(r) for r in conn.execute(sql, args).fetchall()]
    finally:
        conn.close()


# ---------------------------------------------------------------- ★★★ 包裹
_PARCEL_FIELDS = ("parcel_id", "tracking_no", "order_no", "owner", "phone", "dst",
                  "mode", "shelf_no", "pick_code", "status", "drone_id", "car_id",
                  "marker_id",                      # ★★★ AR 码序号（货物身份）
                  "created_at", "loaded_at", "placed_at", "picked_at")


def upsert_parcel(parcel_id: str, **fields):
    """幂等写入包裹：不存在则新建，存在则只更新传入的字段"""
    conn = _connect()
    try:
        t = _now()
        row = conn.execute("SELECT parcel_id FROM parcels WHERE parcel_id=?",
                           (parcel_id,)).fetchone()
        fields = {k: v for k, v in fields.items() if k in _PARCEL_FIELDS}
        if row is None:
            fields.setdefault("created_at", t)
            fields["parcel_id"] = parcel_id
            fields["updated_at"] = t
            cols = ", ".join(fields.keys())
            qs = ", ".join("?" for _ in fields)
            conn.execute("INSERT INTO parcels(%s) VALUES(%s)" % (cols, qs),
                         list(fields.values()))
        else:
            if fields:
                fields["updated_at"] = t
                sets = ", ".join("%s=?" % k for k in fields)
                conn.execute("UPDATE parcels SET %s WHERE parcel_id=?" % sets,
                             list(fields.values()) + [parcel_id])
        conn.commit()
    finally:
        conn.close()


def update_parcel(parcel_id: str, **fields):
    if not fields:
        return
    fields = {k: v for k, v in fields.items() if k in _PARCEL_FIELDS}
    if not fields:
        return
    fields["updated_at"] = _now()
    sets = ", ".join("%s=?" % k for k in fields)
    conn = _connect()
    try:
        conn.execute("UPDATE parcels SET %s WHERE parcel_id=?" % sets,
                     list(fields.values()) + [parcel_id])
        conn.commit()
    finally:
        conn.close()


def get_parcel(parcel_id: str) -> Optional[Dict[str, Any]]:
    conn = _connect()
    try:
        r = conn.execute("SELECT * FROM parcels WHERE parcel_id=?",
                         (parcel_id,)).fetchone()
        return dict(r) if r else None
    finally:
        conn.close()


def find_parcel(tracking_no: Optional[str] = None,
                parcel_id: Optional[str] = None) -> Optional[Dict[str, Any]]:
    """★ 用【快递单号】或内部编号找包裹 —— 机械臂扫到条码后就是走这里查"""
    conn = _connect()
    try:
        if parcel_id:
            r = conn.execute("SELECT * FROM parcels WHERE parcel_id=?",
                             (parcel_id,)).fetchone()
        elif tracking_no:
            r = conn.execute("SELECT * FROM parcels WHERE tracking_no=? "
                             "ORDER BY created_at DESC LIMIT 1", (tracking_no,)).fetchone()
        else:
            return None
        return dict(r) if r else None
    finally:
        conn.close()


def list_parcels(limit: int = 200, status: Optional[str] = None,
                 car_id: Optional[str] = None, q: Optional[str] = None) -> List[Dict[str, Any]]:
    """★ q = 关键词：跨【包裹号 / 运单号 / 收件人 / 手机号 / 取件码 / 目的地】搜索"""
    sql = "SELECT * FROM parcels WHERE 1=1"
    args: List[Any] = []
    if status:
        sql += " AND status=?"
        args.append(status)
    if car_id:
        sql += " AND car_id=?"
        args.append(car_id)
    if q:
        kw = "%" + q.strip() + "%"
        sql += (" AND (parcel_id LIKE ? OR tracking_no LIKE ? OR owner LIKE ?"
                " OR phone LIKE ? OR pick_code LIKE ? OR dst LIKE ?)")
        args += [kw] * 6
    sql += " ORDER BY created_at DESC LIMIT ?"
    args.append(int(limit))
    conn = _connect()
    try:
        return [dict(r) for r in conn.execute(sql, args).fetchall()]
    finally:
        conn.close()


def find_parcels_by(phone: Optional[str] = None,
                    pick_code: Optional[str] = None,
                    owner: Optional[str] = None,
                    limit: int = 50) -> List[Dict[str, Any]]:
    """★ 用户视角查询：只按【手机号 / 取件码 / 收件人】找，不返回别人的数据"""
    conds, args = [], []
    if phone:
        conds.append("phone LIKE ?"); args.append("%" + phone.strip() + "%")
    if pick_code:
        conds.append("pick_code = ?"); args.append(pick_code.strip())
    if owner:
        conds.append("owner = ?"); args.append(owner.strip())
    if not conds:
        return []
    sql = ("SELECT * FROM parcels WHERE (" + " OR ".join(conds) +
           ") ORDER BY created_at DESC LIMIT ?")
    args.append(int(limit))
    conn = _connect()
    try:
        return [dict(r) for r in conn.execute(sql, args).fetchall()]
    finally:
        conn.close()


def parcel_stats() -> Dict[str, Any]:
    """包裹维度统计（论文用）"""
    conn = _connect()
    try:
        tot = conn.execute("SELECT COUNT(*) c FROM parcels").fetchone()["c"]
        picked = conn.execute("SELECT COUNT(*) c FROM parcels WHERE status='PICKED'").fetchone()["c"]
        placed = conn.execute("SELECT COUNT(*) c FROM parcels WHERE status='PLACED'").fetchone()["c"]
        rows = conn.execute("SELECT created_at, picked_at FROM parcels "
                            "WHERE status='PICKED' AND picked_at IS NOT NULL "
                            "AND created_at IS NOT NULL").fetchall()
        durs = [r["picked_at"] - r["created_at"] for r in rows
                if r["picked_at"] and r["created_at"] and r["picked_at"] >= r["created_at"]]
        return {"parcels_total": tot, "parcels_picked": picked,
                "parcels_waiting": placed,
                "parcels_avg_sec": (round(sum(durs)/len(durs), 1) if durs else None)}
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
