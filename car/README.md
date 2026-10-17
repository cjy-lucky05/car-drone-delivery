# 小车端源码（ROS 节点 + STM32）

本目录 = 毕设【小车端】的**可读源码**，从真机 `rockpi-5b` 上拷下来的，与实机运行版本一致。

> ★ 说明：**厂家 SDK、第三方库、编译产物**没有放进来（体积大 / 版权不属于本项目 / 可重新生成）。
> 需要跑起来时，按下文「缺失项」各自补装即可。

---

## 一、目录结构

```
car/
├── ros_ws/src/                        ROS1 Noetic 工作空间源码
│   ├── wd_base_control/               底盘控制（串口 ↔ /cmd_vel，读里程计/电池/IMU）
│   │   ├── src/                       本项目的节点源码
│   │   ├── include/                   头文件（含第三方 HTTP 库 pistache 头，编译必需）
│   │   ├── launch/                    base_control.launch
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── wd_robot_navigation/           导航（雷达 + AMCL + move_base/TEB + RViz）
│   │   ├── launch/                    robot_navigation.launch 等
│   │   ├── param/                     TEB/costmap 参数 yaml
│   │   ├── maps/                      ★ 建图结果（含毕设在用的 lab_20261009）
│   │   ├── rviz/ · config/ · src/
│   ├── wd_vision/                     相机 + AR 识别（usb_cam + ar_track_alvar 封装）
│   │   ├── launch/ar_track.launch     ★ max_new_marker_error 必须 0.08
│   │   └── MarkerData_3/4/5.png       AR 码图（id = 3 / 4 / 5，即"货物身份证"）
│   ├── arm_moveit_config/             机械臂 MoveIt 配置
│   │   └── config/arm.srdf            ★ 姿态表（sentry / grap / init 的弧度值）
│   └── wd_arm_moveit_demo/            ★ 机械臂抓取 Demo（源自学长 王明辉 的毕设，见「来源」）
│       ├── src/ar_pick_place.cpp      原版抓取流程（参考用）
│       └── launch/ · msg/ · rviz/
└── stm32/                             STM32 下位机固件源码（底盘）
    ├── Core/                          ★ 真正的项目代码（main.c / 串口 / 电机 / 里程计）
    │   ├── Inc/
    │   └── Src/
    └── wd_car.ioc                     CubeMX 工程配置（引脚/时钟/外设都在这里）
```

---

## 二、各部分与毕设的关系

| 目录 | 在毕设里干什么 | 谁写的 |
|---|---|---|
| `wd_base_control` | 底盘驱动：把 `/cmd_vel` 变成串口指令，回读里程计/电压/IMU 角速度 | 冰达（厂家）提供 + 少量本项目改动 |
| `wd_robot_navigation` | 导航栈配置 + **自己建的地图** + TEB 调参 | 本项目（基于 ROS navigation 官方包） |
| `wd_vision` | 相机驱动封装 + AR 码识别参数（★ 抓取前用来核对"货物身份证"） | 本项目（封装 usb_cam + ar_track_alvar） |
| `arm_moveit_config` | 机械臂运动学/姿态配置（sentry / grap 两套姿态就在 `arm.srdf`） | 基于厂家配置改 |
| `wd_arm_moveit_demo` | 机械臂抓取示范流程（`ar_pick_place.cpp`） | **王明辉（学长）毕设**，本项目参考/复用 |
| `stm32/Core` | 底盘下位机固件：电机 PWM、编码器里程计、串口协议、电池电压 | 冰达（厂家）提供 |

★ 毕设**自己写的主要是**：桥接节点 `mqtt_bridge_node.py`（在仓库 `设备端/`）、
视觉对准 `car_align.py`（`设备端/`）、导航调参与地图、AR 码校验逻辑、以及平台侧全部代码。

---

## 三、★ 缺失项（没传，需要时自己补）

| 缺什么 | 为什么没传 | 怎么补 |
|---|---|---|
| `bingda_ros1_noetic/`（71M） | 冰达厂家 SDK，版权属厂家 | 随车出厂已装，或向厂家索取 |
| `arm_description/`（78M） | 机械臂 URDF + mesh 模型（厂家提供，大文件） | 随车已装；或从厂家 SDK 取 |
| `ar_track_alvar/`（2.6M） | ROS 官方第三方包 | `sudo apt install ros-noetic-ar-track-alvar` |
| `wd_base_control/lib/`（26M） | 编译产物（.so） | `catkin_make` 重新生成 |
| `stm32/Drivers/`（112M） | ST 官方 HAL 库 | STM32CubeMX 生成，或从 ST 官网下载 |
| `stm32/MDK-ARM/`（65M） | Keil 编译中间产物 | 用 Keil 打开 `wd_car.ioc` 重新编译 |

---

## 四、★ 怎么用（在真机上）

```bash
# 1) 把 ros_ws/src 里的包放进工作空间
cp -r car/ros_ws/src/* ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make

# 2) 启动（★ 4 个终端，别单独起 base_control —— 导航 launch 自带）
source /opt/ros/noetic/setup.bash && roscore
source ~/catkin_ws/devel/setup.bash && roslaunch wd_robot_navigation robot_navigation.launch \
    open_rviz:=true map_file:=$HOME/chen_car_drone/maps/lab_20261009.yaml
source ~/catkin_ws/devel/setup.bash && roslaunch wd_vision bringup.launch
cd ~/chen_car_drone/小车 && source ~/catkin_ws/devel/setup.bash && python3 -u mqtt_bridge_node.py
```

```bash
# 3) STM32 固件：用 STM32CubeMX 打开 wd_car.ioc，或用 Keil 打开 MDK-ARM/wd_car.uvprojx
#    （需先补 Drivers/ 目录）
```

---

## 五、★ 来源与版权说明（如实记录）

- `wd_base_control` / `arm_moveit_config` / `arm_description` / `stm32` —— **冰达智能（Bingda）**
  随车提供的官方 SDK 与本项目在其基础上的改动。
- `wd_vision` —— 本项目对 `usb_cam` + `ar_track_alvar`（ROS 官方）的封装与参数调整。
- `wd_arm_moveit_demo` —— **王明辉** 学长毕设的机械臂抓取流程，本项目参考并复用其思路
  （本毕设的机械臂部分不是本人成果，如实注明）。
- ★ **亚博智能（Yahboom / Dofbot）官方教程正文与资料未包含在本仓库** ——
  那是第三方教学资料，版权属亚博，仅在本地查阅，不上传。

---

## 六、★ 实机验证过的事实（供复现参考）

- 底盘最小转弯半径 `min_turning_radius = 0.375 m`，**不能原地转向**；
  命令曲率 `|w|/|v| > 2.67` 时底盘只打前轮、后轮不走（车纹丝不动）。
- 底盘要**持续收到** `/cmd_vel` 才走（20Hz 连发）；裸发一次不生效。
- 轮径在固件里写死 64.0 mm，改轮径要改固件而不是改 UI。
- IMU：`orientation` 是无效哨兵值（65534），**姿态不可用**；角速度与加速度正常。
- AR 码识别（`ar_track_alvar`）：`max_new_marker_error=0.08`；视角斜到 30~45° 会解不出 ID，
  识别是断续的（实测约 6% 的帧有码）。

---
*本目录内容随实机代码同步；最后同步：2026-10-17*
