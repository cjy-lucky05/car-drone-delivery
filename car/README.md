# 小车端源码（ROS 节点 + STM32 固件）

本目录 = 毕设【小车端】的**完整可读源码**，从真机 `rockpi-5b` 上同步，与实机运行版本一致。

> 完整覆盖：ROS 工作空间全部相关包（含厂商 SDK、模型、地图）+ STM32 下位机固件（含 HAL 驱动、Keil 工程）。

---

## 一、目录结构

```
car/
├── ros_ws/src/                        ROS1 Noetic 工作空间
│   ├── wd_base_control/               ★ 底盘驱动层
│   │   ├── src/  include/             本项目节点源码 + 头文件
│   │   ├── lib/                       运行库（含 MobileNet-SSD 检测模型）
│   │   ├── launch/  CMakeLists.txt  package.xml
│   ├── wd_robot_navigation/           ★ 导航层
│   │   ├── launch/                    robot_navigation.launch（★ 自带底盘节点）
│   │   ├── param/                     TEB / costmap 参数
│   │   ├── maps/                      SLAM 建图结果（含 lab_20261009）
│   │   ├── rviz/  config/  src/
│   ├── wd_vision/                     ★ 视觉层
│   │   ├── launch/ar_track.launch     相机 + AR Tag 识别参数
│   │   └── MarkerData_3/4/5.png       AR Tag 图（id = 3/4/5，货物标识）
│   ├── arm_moveit_config/             机械臂运动学与规划配置
│   │   └── config/arm.srdf            ★ 姿态表（init / sentry / grap）
│   ├── wd_arm_moveit_demo/            机械臂抓取流程实现
│   │   └── src/ar_pick_place.cpp      MoveIt action 客户端 + 抓取状态机
│   ├── arm_description/               机械臂 URDF + 3D 模型（STL）
│   ├── ar_track_alvar/                AR Tag 检测库（ROS 封装）
│   └── bingda_ros1_noetic/            底盘厂商 ROS 驱动包
└── stm32/                             ★ STM32 下位机固件（底盘）
    ├── Core/                          项目代码：main.c / 电机 / 编码器 / 串口 / 舵机
    ├── Drivers/                       ST HAL 库 + CMSIS
    ├── MDK-ARM/                       Keil 工程（可直接打开编译）
    └── wd_car.ioc                     STM32CubeMX 工程配置
```

---

## 二、技术栈与算法基础（各层的学术来源）

| 层 | 技术方案 | 算法 / 理论来源 |
|---|---|---|
| 定位 | **AMCL** 自适应蒙特卡洛定位 | Thrun, Fox, Burgard, Dellaert — *Robust Monte Carlo Localization for Mobile Robots* (2001)；ROS 实现采用 KLD-sampling 自适应粒子数 |
| 全局规划 | **NavFn / global_planner** | Dijkstra / A* 栅格搜索（ROS Navigation Stack） |
| 局部规划 | **TEB**（Timed Elastic Band） | Rösmann et al. — *Integrated online trajectory planning and optimization in distinctive topologies* (2017)。把局部规划建模为**带时间参数的非线性优化问题**，直接约束车辆运动学（含最小转弯半径） |
| 代价地图 | layered costmap | 静态层 / 障碍层 / 膨胀层叠加，膨胀半径决定可行走廊宽度 |
| 视觉定位 | **AR Tag 位姿估计**（ALVAR 库） | VTT Technical Research Centre of Finland / University of Oulu — ALVAR。流程：四边形检测 → 图案解码 → PnP 解算 6-DoF 位姿 |
| 机械臂规划 | **MoveIt + OMPL + KDL IK** | OMPL 采样规划（RRT*/PRM）；KDL 逆运动学求解 |
| 下位机 | STM32F3 + HAL | 编码器正交解码测速 → 里程计积分；串口协议上下行 |

---

## 三、★ 本项目自主设计与实现的部分

> 这一节是本毕设的**技术贡献点**，其余为在其上集成的开源/厂商组件。

### 1. 跨 ROS 版本 / 跨网络通信桥接（`设备端/mqtt_bridge_node.py`）
- 把 MQTT 指令翻译成 ROS 动作，向上屏蔽 ROS1/ROS2 与网络的差异；
- 指令**状态机**（导航中 / 等待入库 / 抓取中 / 送达中 / 待命），带幂等与阶段保护；
- 上报事件与平台共享同一套语义（idle / arrived / car_load / shelf_place / exception…）。

### 2. AR Tag 作为"货物身份证"的**双端校验机制**
- 平台派单时下发期望的 Tag 号，小车抓取前必须读到**同一个号**才抓，不一致则拒抓并告警；
- 解决"无人机放错库位 / 中途调包 / 抓空气"三类失效，属于本项目的**业务级防错设计**。

### 3. 视觉伺服对准算法（`设备端/car_align.py` → 桥接节点 `_visual_align`）
- **目标值示教**：将"夹爪刚好能夹住"时的 Tag 在**相机坐标系**读数 (x, y) 作为伺服目标 ——
  该目标只与相机安装、机械臂姿态、物料尺寸有关，**与车停在哪无关**，因此姿态固定后即为常量；
- **控制律**：横向偏差 → 角速度（车头转角与图像横移角近似 1:1），纵向偏差 → 前进/后退，
  两者同时输出即沿**小弧线**收敛；
- **阿克曼可行性裁剪**：显式施加曲率约束 `|ω|/|v| ≤ 1/R_min`（实测 R_min=0.375 m），
  并预留 15% 余量；为保证底盘响应，指令需按 20 Hz 连续下发设定时长（裸发单帧不生效）；
- **实测**：横向/纵向初始偏差 54 mm / 46 mm → 13 步收敛至 7.7 mm / 6.7 mm，随后夹取成功。

### 4. 停位误差的**闭环阻尼补偿**（桥接节点 `_dock_to_site`）
- 问题：导航栈停位误差实测 ±6~12 cm 且**具有系统性**（同向偏），而夹爪容差仅约 1.5 cm；
- 方法：读 AMCL 位姿与目标比对，按 `目标' = 目标 − k·(实测 − 目标)` 迭代逼近；
  **k=1.0 会过冲并振荡**（实测 −31 cm → +21 cm），故取 **k=0.6 并加入过冲自适应**（偏差变大则 k 减半）；
- 效果：一次 31.8 cm 的偏差在第 2 轮即进入 8 cm 容差，之后交给视觉伺服收尾。

### 5. 车-机协同调度平台（`平台/hub.py` + `web平台/`）
- 空地协同的任务派发：无人机投放事件 → 建包裹 → 生成取货任务 → 按站点 FIFO 队列派车；
- **数据一致性约束**：包裹必须绑定已注册用户（手机号校验 + 以用户表真名覆盖），
  杜绝"孤儿包裹"；货架占用状态以包裹表为唯一真相源重算；
- 前端不写库，人的操作只发 MQTT，平台为**唯一写入者**（单一真相源，零并发写）。

---

## 四、★ 实机验证过的关键事实（复现参考）

- 底盘 `min_turning_radius = 0.375 m`，**不能原地转向**；命令曲率超过 `1/0.375 ≈ 2.67` 时
  底盘只打前轮、后轮不走（现象是"车纹丝不动"）。
- `/cmd_vel` 必须**持续下发**（20 Hz）；单帧发布底盘不响应。
- 轮径在固件中写死 64.0 mm，改轮径须改固件而非只改上位机。
- IMU 的 `orientation` 为无效哨兵值（65534），**姿态不可用**；角速度与加速度正常 →
  只用角速度，姿态由上位机自行解算。
- AR Tag 识别（`max_new_marker_error = 0.08`）：视角斜至 30~45° 时解码失败；
  识别呈**间歇性**（实测约 6% 的帧成功），故判据须用"时间窗"而非单帧。
- 导航停位误差 ±6~12 cm，且偏在同一方向（系统性）→ 可用闭环补偿抵消。

---

## 五、怎么跑

```bash
# 1) 编译
cp -r car/ros_ws/src/* ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make

# 2) 启动（★ 4 个终端；导航 launch 自带底盘，不要再单独起 base_control）
source /opt/ros/noetic/setup.bash && roscore

source ~/catkin_ws/devel/setup.bash && roslaunch wd_robot_navigation robot_navigation.launch \
    open_rviz:=true map_file:=$HOME/chen_car_drone/maps/lab_20261009.yaml

source ~/catkin_ws/devel/setup.bash && roslaunch wd_vision bringup.launch

cd ~/chen_car_drone/小车 && source ~/catkin_ws/devel/setup.bash && python3 -u mqtt_bridge_node.py
```

```bash
# 3) STM32 固件：STM32CubeMX 打开 wd_car.ioc，或 Keil 打开 stm32/MDK-ARM/wd_car.uvprojx
```

---

## 六、来源与许可说明

- 底盘驱动（`bingda_ros1_noetic`、`wd_base_control` 的底层部分）、机械臂 URDF 与模型
  （`arm_description`）、STM32 HAL 库（`Drivers`）来自**底盘与机械臂厂商提供的 SDK**，版权归厂商。
- `ar_track_alvar` 为开源 ROS 包（BSD），`MoveIt` / `Navigation Stack` / `OMPL` / `KDL` 为 ROS 开源生态组件。
- `wd_arm_moveit_demo`、`arm_moveit_config` 等在实验室既有代码与厂商配置基础上改造，
  改造内容（姿态表、抓取状态机、参数）随本项目演进，见 git 历史。
- 本项目**自主实现**的部分见第三节；★ 本仓库**不包含**任何第三方教学资料正文
  （厂商教程正文、课程资料等）—— 那类资料版权归属各自所有方，仅在本地查阅。

---
*最后同步：2026-10-17（与真机 `rockpi-5b` 一致）*
