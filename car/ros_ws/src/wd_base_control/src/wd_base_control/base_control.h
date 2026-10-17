// #ifndef __BASE_CONTROL_H__
// #define __BASE_CONTROL_H__

// #include "ros/ros.h"
// #include <string>
// #include <vector>
// #include <ros/time.h>
// #include <geometry_msgs/Vector3.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/Twist.h>
// #include <fcntl.h>
// #include "queue.h"
// #include <cstdlib>
// #include <termios.h>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/BatteryState.h>
// #include <std_msgs/UInt8MultiArray.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/Range.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>
// #include <signal.h>
// #include <std_msgs/ByteMultiArray.h>
// #include <wd_arm_moveit_demo/ArmJoint.h>

// // 移动平台底层控制类
// class BaseControl
// {
// public:
//     // 构造函数：传入 ROS NodeHandle 指针
//     BaseControl(ros::NodeHandle *phandle);
    
//     // 析构函数
//     ~BaseControl();
//     void sendTestFrame();
//     // 订阅/cmd_vel消息的回调函数，用于控制底盘移动
//     void cmdCB(const geometry_msgs::Twist::ConstPtr data);

//     // 订阅/serial_data消息的回调函数，用于接收并转发串口数据
//     void serialDataCB(const std_msgs::UInt8MultiArray::ConstPtr& msg);
//     // 定时器回调：通信检查
//     void timerCommunicationCB(const ros::TimerEvent &event);

//     // 定时器回调：发布里程计信息
//     void timerOdomCB(const ros::TimerEvent &event);

//     // 定时器回调：发布电池状态
//     void timerBatteryCB(const ros::TimerEvent &event);

//     // 定时器回调：发布 IMU 数据
//     void timerIMUCB(const ros::TimerEvent &event);

//     // 定时器回调：发布超声波数据（函数名疑似拼写错误）
//     void timetimerSonarCBrIMUCB(const ros::TimerEvent &event);
    
//     // 获取固件版本号
//     void getVersion();

//     // 获取序列号
//     void getSN();

//     // 获取底盘基本信息（类型、电机型号等）
//     void getInfo();

//     int arm_convert_value(int id, double angle);
//     int arm_convert_angle(int id, double s_value);
//     void set_uart_servo_angle(unsigned int s_id, double s_angle, unsigned int run_time);
//     void set_uart_servo_torque(unsigned int enable);
//     void set_uart_servo_angle_array(const std::vector<float> &angle_s, unsigned int run_time);
//     void get_uart_servo_value(unsigned int s_id);
//     int get_uart_servo_angle(unsigned int s_id);
//     int get_uart_servo_angle_array(std::vector<double> &angle);
//     void Armcallback(const wd_arm_moveit_demo::ArmJoint::ConstPtr msg);

// private:
//     // NodeHandle 指针
//     ros::NodeHandle *phandle;

//     // 底盘/IMU/电池配置参数
//     std::string baseId;
//     std::string device_port;
//     int baudrate;         // 串口波特率
//     int serial;           // 串口文件描述符

//     // 里程计发布配置
//     std::string odomId;
//     std::string odom_topic;
//     int odom_freq;

//     // 电池发布配置
//     std::string battery_topic;
//     float battery_freq;

//     // 速度控制输入话题
//     std::string cmd_vel_topic;
//     std::string ackermann_cmd_topic;
//     std::string arm_topic;
//     std::string jointstate_topic;

//     // IMU 发布配置
//     bool pub_imu;
//     std::string imuId;
//     std::string imu_topic;
//     float imu_freq;

//     // 超声波发布标志
//     bool pub_sonar;

//     // 是否启用 ackermann steering 模式
//     bool sub_ackermann;

//     // 串口数据话题
//     std::string serial_data_topic;

//     // 时间记录
//     ros::Time current_time;
//     ros::Time previous_time;

//     // 位姿与速度变量
//     float pose_x;
//     float pose_y;
//     float pose_yaw;
//     float trans_x;    // linear.x
//     float trans_y;    // linear.y
//     float rotat_z;    // angular.z
//     float speed;
//     float steering_angle;

//     // 串口状态
//     int serialIDLE_flag;
//     int sendcounter;

//     // 各种错误标志
//     bool ImuErrFlag = false;
//     bool EncoderFlag = false;
//     bool BatteryFlag = false;

//     // 时间统计器
//     int OdomTimeCounter = 0;
//     int BatteryTimeCounter = 0;

//     // 底层通信数据队列
//     wd::Queue *queue;

//     // 实际底盘速度反馈值（单位：int16）
//     int16_t Vx = 0;     // 线速度x
//     int16_t Vy = 0;     // 线速度y
//     int16_t Vyaw = 0;   // 角速度yaw
//     int16_t Yawz = 0;   // Yaw角
//     int16_t Vvoltage = 0;  // 电压
//     int16_t Icurrent = 0;  // 电流

//     // IMU 数据
//     geometry_msgs::Vector3 Gyro;
//     geometry_msgs::Vector3 Accel;
//     geometry_msgs::Quaternion Quat;

//     // 超声波距离数组（4个方向）
//     char Sonar[4];

//     // 版本信息
//     char movebase_firmware_version[3];
//     char movebase_hardware_version[3];
//     std::string movebase_type[3];
//     std::string motor_type[3];

//     // 控制命令的最后接收时间
//     ros::Time last_cmd_vel_time;
//     ros::Time last_ackermann_cmd_time;

//     // ROS 通信组件
//     ros::Subscriber cmd_sub;  // 订阅控制消息
//     ros::Subscriber serial_sub;  // 订阅串口数据
//     ros::Subscriber arm_sub;
//     ros::Publisher odom_pub, battery_pub, imu_pub;  // 发布状态信息
//     tf::TransformBroadcaster tf_broadcaster; // 发布TF变换

//     int read_id;
//     unsigned int read_val;
//     int read_arm[6];
//     int read_arm_ok;

//     // 各种定时器
//     ros::Timer timer_odom, timer_battery, timer_communication, timer_imu;
// };

// #endif










#ifndef __BASE_CONTROL_H__
#define __BASE_CONTROL_H__

#include "ros/ros.h"
#include <string>
#include <vector>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <fcntl.h>
#include "queue.h"
#include <cstdlib>
#include <termios.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <signal.h>
#include "wd_arm_moveit_demo/ArmJoint.h"
class BaseControl
{
public:
    BaseControl(ros::NodeHandle *phandle);
    ~BaseControl();
    void cmdCB(const geometry_msgs::Twist::ConstPtr data);
    void Armcallback(const wd_arm_moveit_demo::ArmJoint::ConstPtr msg);
    void timerCommunicationCB(const ros::TimerEvent &event);
    void timerOdomCB(const ros::TimerEvent &event);
    void timerBatteryCB(const ros::TimerEvent &event);
    void timerIMUCB(const ros::TimerEvent &event);
    void timetimerSonarCBrIMUCB(const ros::TimerEvent &event);
    
    void getVersion();
    void getSN();
    void getInfo();

    int arm_convert_value(int id, double angle);
    int arm_convert_angle(int id, double s_value);
    void set_uart_servo_angle(unsigned int s_id, double s_angle, unsigned int run_time);
    void set_uart_servo_torque(unsigned int enable);
    void set_uart_servo_angle_array(const std::vector<float> &angle_s, unsigned int run_time);
    void get_uart_servo_value(unsigned int s_id);
    int get_uart_servo_angle(unsigned int s_id);
    int get_uart_servo_angle_array( std::vector<double> &angle);
   
private:
    ros::NodeHandle *phandle;
    std::string baseId;

    std::string device_port;
    int baudrate;
    int serial;

    std::string odomId;
    std::string odom_topic;
    int odom_freq;

    std::string battery_topic;
    float battery_freq;

    std::string cmd_vel_topic;
    std::string ackermann_cmd_topic;

    bool pub_imu;
    std::string imuId;
    std::string imu_topic;
    float imu_freq;

    std::string arm_topic;
    std::string jointstate_topic;


    bool pub_sonar;
    bool sub_ackermann;
    ros::Time current_time;
    ros::Time previous_time;
    float pose_x;
    float pose_y;
    float pose_yaw;
    int serialIDLE_flag;
    float trans_x; // linear.x
    float trans_y; // linear.y
    float rotat_z; // angular.z
    float speed;
    float steering_angle;

    int sendcounter;

    bool ImuErrFlag = false;
    bool EncoderFlag = false;
    bool BatteryFlag = false;
    int OdomTimeCounter = 0;
    int BatteryTimeCounter = 0;

    wd::Queue *queue;
    int16_t Vx = 0;   // real_vx
    int16_t Vy = 0;   // real_Vy
    int16_t Vyaw = 0; // real_Vyaw
    int16_t Yawz = 0;
    int16_t Vvoltage = 0;
    int16_t Icurrent = 0;

    geometry_msgs::Vector3 Gyro;
    geometry_msgs::Vector3 Accel;
    geometry_msgs::Quaternion Quat;
    char Sonar[4];
    char movebase_firmware_version[3];
    char movebase_hardware_version[3];
    std::string movebase_type[3];
    std::string motor_type[3];

    ros::Time last_cmd_vel_time;
    ros::Time last_ackermann_cmd_time;

    ros::Subscriber cmd_sub,arm_sub;
    ros::Publisher odom_pub, battery_pub, imu_pub,jointstate_pub;
    tf::TransformBroadcaster tf_broadcaster;
    ros::Timer timer_jointstate,timer_odom, timer_battery, timer_communication, timer_imu;

    unsigned int  read_id,read_val;
    char read_arm_ok;
    unsigned int read_arm[6];
};

#endif
