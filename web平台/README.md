# web平台 · 车-机协同末端配送监控台

网页版监控台：看库位/任务/统计 + 手动下发指令。

## 目录结构

```
web平台/
├── server/                后端
│   ├── app.py             ★ FastAPI 主程序（提供 HTTP 接口）
│   ├── db.py              ★ 数据库访问层（SQLite）
│   ├── schema.sql         ★ 建表语句（5 张表）
│   ├── mqtt_cmd.py        下发 MQTT 指令
│   └── config.example.py  配置模板（复制成 config_local.py 填密码）
├── static/
│   └── index.html         ★ 前端页面（单文件，深色工业风）
├── scripts/
│   ├── start_web.sh       一键启动
│   └── stop_web.sh        一键停止
├── data/                  SQLite 数据库文件（不进仓库）
└── requirements.txt
```

## 架构（谁干什么）

```
小车 / 无人机 ──MQTT──▶ broker ──▶ hub.py（大脑 · 唯一写库者）──▶ SQLite
                                                                    ▲
人（浏览器）──HTTP──▶ FastAPI ────────────────────────────────────────┘（只读）
                        │
                        └── 人的操作 → MQTT 消息 → hub.py 决策
```

★ 设计要点：**前端不直接写数据库** —— 人的操作以 MQTT 消息发给 hub.py，
由它统一决策 + 写库。这样 hub.py 始终是**唯一写入者**，不存在并发写问题。

## 数据库（5 张表）

| 表 | 用途 |
|---|---|
| `slots` | 库位状态（1A 有没有货）|
| `tasks` | 任务（含 created/assigned/picked/delivered 四个时间戳 → ★ 能算耗时）|
| `queue` | 派单队列（按站点分，先进先出）|
| `devices` | 设备在线状态 |
| `events` | ★ 事件日志（每次上报记一行 → 论文统计的数据源）|

## 部署（服务器上）

```bash
# ① 装依赖（不动系统 Python）
pip3 install --target=/root/chenjiayu/libs fastapi "uvicorn[standard]" paho-mqtt

# ② 放配置（含 MQTT 密码）
cp /root/chenjiayu/web平台/server/config.example.py /root/chenjiayu/web平台/server/config_local.py
# 然后编辑 config_local.py 填密码

# ③ 启动
bash /root/chenjiayu/web平台/scripts/start_web.sh
```

浏览器访问 `http://<你的服务器IP>:8000`（★ 记得在云控制台安全组放行 8000 端口）

## 接口一览

| 接口 | 说明 |
|---|---|
| `GET /` | 网页 |
| `GET /api/status` | ★ 网页轮询用（库位+任务+设备+统计）|
| `GET /api/tasks` | 任务列表（可带 limit/site/status）|
| `GET /api/events` | 事件日志 |
| `GET /api/stats` | ★ 统计（完成数/成功率/平均耗时）|
| `POST /api/cmd` | 下发指令（转成 MQTT 消息）|
| `GET /docs` | ★ FastAPI 自带接口文档（可点着测）|

## 本地测试

```bash
cd server && python3 db.py        # 建库 + 打印统计
python3 app.py                    # 启动（浏览器 localhost:8000）
```
