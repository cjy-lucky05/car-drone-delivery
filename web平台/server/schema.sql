-- ============================================================
--  车-机协同末端配送系统 · 数据库表结构（SQLite）
-- ============================================================
--  五张表：
--    slots   库位      （1A 现在有没有货）
--    tasks   任务      （谁送的、什么状态、花了多久）
--    queue   派单队列  （按站点分开，先进先出）
--    devices 设备      （小车/无人机/人的在线状态）
--    events  事件日志  （★ 所有上报都记一行 → 统计的数据源）
-- ============================================================

-- 库位
CREATE TABLE IF NOT EXISTS slots (
    slot_id    TEXT PRIMARY KEY,      -- 1A / 1B / 2A ...
    site       TEXT NOT NULL,         -- 所属站点 1/2/3
    occupied   INTEGER DEFAULT 0,     -- 0=空 1=占用
    count      INTEGER DEFAULT 0,     -- 里面有几件
    task_id    TEXT,                  -- 当前存放的任务号
    updated_at INTEGER                -- 更新时间（Unix 秒）
);

-- 任务
CREATE TABLE IF NOT EXISTS tasks (
    task_id      TEXT PRIMARY KEY,    -- T0001
    site         TEXT,                -- 站点
    slot         TEXT,                -- 库位
    count        INTEGER,             -- 件数
    car_id       TEXT,                -- 执行的小车
    status       TEXT,                -- PENDING/ASSIGNED/PICKING/DELIVERING/DONE/FAILED
    created_at   INTEGER,             -- 创建时间
    assigned_at  INTEGER,             -- 派单时间
    picked_at    INTEGER,             -- 取货完成时间
    delivered_at INTEGER,             -- 送达完成时间
    updated_at   INTEGER
);

-- 派单队列（按站点分隔，先进先出）
CREATE TABLE IF NOT EXISTS queue (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,
    site       TEXT NOT NULL,
    task_id    TEXT NOT NULL,
    enqueue_at INTEGER
);

-- 设备
CREATE TABLE IF NOT EXISTS devices (
    device_id TEXT PRIMARY KEY,       -- car-01 / nx-01
    role      TEXT,                   -- car / drone
    status    TEXT,                   -- idle / busy / offline
    last_seen INTEGER,
    -- ★ 实时位置（2026-10-05 新增，供网页地图可视化用）
    pos_x     REAL,                   -- 世界坐标 x（米）
    pos_y     REAL,                   -- 世界坐标 y（米）
    pos_yaw   REAL,                   -- 朝向（弧度）
    pos_ts    INTEGER,                -- 位置更新时间
    -- ★ 设备信息（2026-10-08 新增，供"设备信息"卡显示）
    name      TEXT,                   -- 显示名（如 "小车 1 号"）
    battery   REAL,                   -- 电量百分比 0~100
    battery_v REAL,                   -- 电压（V）
    battery_ts INTEGER,               -- 电量上报时间
    task_now  TEXT                    -- 当前任务（正在运什么）
);

-- ★★★ 包裹（每件货一行）—— 系统的核心表
--     物流公司的轨迹到【校门口/驿站】就断了，本表 + parcel 时间线来补齐"校园段"
CREATE TABLE IF NOT EXISTS parcels (
    parcel_id   TEXT PRIMARY KEY,     -- P0001（系统内部编号）
    tracking_no TEXT,                 -- 快递单号（上游，用于对齐物流公司）
    order_no    TEXT,                 -- 订单号
    owner       TEXT,                 -- 收件人（张三）
    phone       TEXT,                 -- 手机号
    dst         TEXT,                 -- 目的地：3号楼 / 图书馆 / 驿站
    mode        TEXT,                 -- SHELF=送指定货柜  STATION=送驿站
    shelf_no    TEXT,                 -- 目标货柜位置（如 "3号楼-第1层-2格"）
    pick_code   TEXT,                 -- 取件码（4位数字）
    status      TEXT,                 -- CREATED/ON_DRONE/AT_GATE/LOADED/DELIVERING/PLACED/PICKED
    drone_id    TEXT,                 -- 承运无人机
    car_id      TEXT,                 -- 承运小车
    created_at  INTEGER,              -- 揽收时间（中转点）
    loaded_at   INTEGER,              -- 小车装载时间
    placed_at   INTEGER,              -- 放入货柜时间
    picked_at   INTEGER,              -- 取件人取走时间
    updated_at  INTEGER
);

-- ★ 事件日志：设备每一次上报都存一行，是做统计的数据源
--   parcel_id：★ 若该事件属于某个包裹，就填上 → 用它拼出"这件货自己的时间线"
CREATE TABLE IF NOT EXISTS events (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    device    TEXT,
    event     TEXT,
    payload   TEXT,                   -- 原始 JSON
    ts        INTEGER,
    parcel_id TEXT                    -- ★ 属于哪个包裹（用于"按包裹看时间线"）
);

-- 索引（加速常用查询）
CREATE INDEX IF NOT EXISTS idx_tasks_site_status ON tasks(site, status);
CREATE INDEX IF NOT EXISTS idx_tasks_created     ON tasks(created_at);
CREATE INDEX IF NOT EXISTS idx_queue_site        ON queue(site, id);
CREATE INDEX IF NOT EXISTS idx_events_ts         ON events(ts);
CREATE INDEX IF NOT EXISTS idx_events_device     ON events(device, ts);


-- ★★★ 用户（2026-10-08 新增）—— 管理员可增删改；也是"用户登录"的依据
CREATE TABLE IF NOT EXISTS users (
    user_id    TEXT PRIMARY KEY,     -- U0001
    name       TEXT,                 -- 姓名（张三）
    phone      TEXT,                 -- 手机号 ★ 登录账号（唯一）
    building   TEXT,                 -- 楼栋（如 "3号楼"）→ 按楼栋分组
    role       TEXT DEFAULT 'user',  -- user / admin
    password   TEXT,                 -- ★ 密码（sha256 加盐，不存明文）
    note       TEXT,                 -- 备注（宿舍号等）
    active     INTEGER DEFAULT 1,    -- 1=正常 0=停用
    created_at INTEGER,
    updated_at INTEGER
);
CREATE UNIQUE INDEX IF NOT EXISTS idx_users_phone ON users(phone);

-- 包裹相关索引
CREATE INDEX IF NOT EXISTS idx_parcels_status  ON parcels(status);
CREATE INDEX IF NOT EXISTS idx_parcels_track   ON parcels(tracking_no);
CREATE INDEX IF NOT EXISTS idx_parcels_owner   ON parcels(owner);

-- ★ 老库升级用（已经建过的表补字段；SQLite 不支持 IF NOT EXISTS 加列，失败忽略即可）
-- ALTER TABLE events ADD COLUMN parcel_id TEXT;
-- CREATE INDEX IF NOT EXISTS idx_events_parcel ON events(parcel_id, ts);
-- ALTER TABLE devices ADD COLUMN pos_x REAL;
-- ALTER TABLE devices ADD COLUMN pos_y REAL;
-- ALTER TABLE devices ADD COLUMN pos_yaw REAL;
-- ALTER TABLE devices ADD COLUMN pos_ts INTEGER;


-- ============================================================================
--  ★★ 2026-10-08 新增：无人机 GPS 位置 + 轨迹
-- ============================================================================
-- 轨迹点（画"走过的路径"用；小车用世界坐标 x/y，无人机用经纬度 lat/lon）
CREATE TABLE IF NOT EXISTS tracks (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    device_id TEXT,                   -- car-01 / nx-01
    x         REAL,                   -- 世界坐标（小车用）
    y         REAL,
    lat       REAL,                   -- 经纬度（无人机用）
    lon       REAL,
    alt       REAL,
    ts        INTEGER
);
CREATE INDEX IF NOT EXISTS idx_tracks_dev ON tracks(device_id, ts);


-- ============================================================================
--  ★★ 2026-10-08：货架分两类
--    kind='transit' → 校门口【中转货架】（无人机放 → 小车取）
--    kind='dst'     → 校内【配送点货柜】（小车放 → 收件人来取）
-- ============================================================================
-- slots 表加 kind / name 字段（老库由 db._migrate 自动补）
--   kind : transit / dst
--   name : 显示名（如 "2号公寓楼下"）
