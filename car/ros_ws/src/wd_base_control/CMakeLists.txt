cmake_minimum_required(VERSION 3.0.2)
project(wd_base_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## 找 ROS 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2_ros
  nav_msgs
  tf2_geometry_msgs   # 这里要加
  geometry_msgs
  visualization_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  roslib
  wd_arm_moveit_demo
)
## 查找 OpenCV 依赖
find_package(OpenCV REQUIRED)
## 额外依赖
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_library(PISTACHE_LIB pistache PATHS ${CMAKE_SOURCE_DIR}/wd_base_control/lib)
## Pistache 静态库
# find_library(PISTACHE_LIB pistache PATHS ${CMAKE_SOURCE_DIR}/lib)
include_directories(/usr/include)   # Ubuntu 下 nlohmann_json 就装在这里
find_package(Boost REQUIRED COMPONENTS system)

## catkin 配置
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf tf2_ros visualization_msgs
  tf2_geometry_msgs   
  cv_bridge
  roslib
  wd_arm_moveit_demo
  )

## 头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}  # 添加 OpenCV 头文件路径
)

## 库文件路径 (pistache.a)
link_directories(
  ${CMAKE_SOURCE_DIR}/lib
)


## ------------------- 可执行文件 1：HTTP 服务 -------------------
add_executable(http_node
  src/http_node.cpp
  src/AttackLogic.cpp
  src/BufferQueue.cpp
  src/GoalLogic.cpp
  src/HttpServer.cpp
  src/Logger.cpp
  src/ObstacleAvoidance.cpp
  src/PacketCodec.cpp
  src/PatrolLogic.cpp
  src/RobotState.cpp
  src/RosInterface.cpp
  src/StateMachine.cpp
  src/TcpServer.cpp
  src/tf_utils.cpp
  src/UdpServer.cpp
  src/YamlUtils.cpp
  src/TrackLogic.cpp
  src/Protocol.cpp
)

target_link_libraries(http_node
${catkin_LIBRARIES}
${EIGEN3_LIBRARIES}
${PISTACHE_LIB}
  Boost::system
  pthread
  yaml-cpp
  nlohmann_json::nlohmann_json
)

## ------------------- 可执行文件 2：底盘控制 -------------------
add_executable(wd_base_control
  src/wd_base_control/base_control.cpp
  src/wd_base_control/queue.cpp
)

target_link_libraries(wd_base_control
  ${catkin_LIBRARIES}
  pthread
)
add_dependencies(wd_base_control ${catkin_EXPORTED_TARGETS} ${wd_arm_moveit_demo_EXPORTED_TARGETS})

## ------------------- 可执行文件 3：人类跟踪 -------------------
add_executable(human_tracking
  src/Test/human_tracking.cpp
)

target_link_libraries(human_tracking
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}  # 链接 OpenCV 库
  pthread
)