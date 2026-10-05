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
    last_seen INTEGER
);

-- ★ 事件日志：设备每一次上报都存一行，是做统计的数据源
CREATE TABLE IF NOT EXISTS events (
    id      INTEGER PRIMARY KEY AUTOINCREMENT,
    device  TEXT,
    event   TEXT,
    payload TEXT,                     -- 原始 JSON
    ts      INTEGER
);

-- 索引（加速常用查询）
CREATE INDEX IF NOT EXISTS idx_tasks_site_status ON tasks(site, status);
CREATE INDEX IF NOT EXISTS idx_tasks_created     ON tasks(created_at);
CREATE INDEX IF NOT EXISTS idx_queue_site        ON queue(site, id);
CREATE INDEX IF NOT EXISTS idx_events_ts         ON events(ts);
CREATE INDEX IF NOT EXISTS idx_events_device     ON events(device, ts);
