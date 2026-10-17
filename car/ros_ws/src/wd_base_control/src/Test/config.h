#ifndef CONFIG_H
#define CONFIG_H

// true 表示使用真实摄像头，false 表示仿真环境使用话题
#define USE_REAL_CAMERA false

        // 加载 MobileNetSSD 模型路径（根据实际环境调整）
#define PROTOTXT_PATH "/home/bye/ros_car/src/wd_base_control/lib/trained_network/MobileNetSSD_deploy.prototxt.txt"
#define MODEL_PATH "/home/bye/ros_car/src/wd_base_control/lib/trained_network/MobileNetSSD_deploy.caffemodel"

// 控制参数
const double K_ROTATION = 0.002;
const double K_VELOCITY = 0.0000045;
const double MAX_VELOCITY = 0.25;

// 避障参数
const double OBSTACLE_DISTANCE_THRESHOLD = 0.5;  // 障碍物距离阈值（米）
const double OBSTACLE_SECTOR_ANGLE = 30.0;  // 避障扇区半角（度，±30°）

// 其他常量
const int MIN_AREA_THRESHOLD = 5000;  // 降低面积阈值
const double ASSUMED_DISTANCE_TO_PERSON = 50000.0;  // 调整距离假设
const int DETECT_INTERVAL = 3;  // 缩短检测间隔

// 平滑滤波参数
const double ALPHA = 0.3;  // 低通滤波系数
const double DECELERATION_FACTOR = 0.9;  // 无目标时减速因子

#endif // CONFIG_H