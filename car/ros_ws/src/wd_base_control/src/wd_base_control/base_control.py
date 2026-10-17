#!/usr/bin/python
# coding=gbk
import os
import rospy
import tf
import time
import sys
import math
import serial
import string
import ctypes
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range

# 代码的主要功能是通过ROS与机器人底盘硬件通信，处理速度指令、传感器数据（IMU、超声波、电池等）并发布到ROS话题，同时广播TF变换以支持机器人导航和定位。

# 获取环境变量 BASE_TYPE 和 SONAR_NUM
base_type = os.getenv('BASE_TYPE')  # 底盘类型
if os.getenv('SONAR_NUM') is None:
    sonar_num = 0  # 默认无超声波传感器
else:
    sonar_num = int(os.getenv('SONAR_NUM'))  # 超声波传感器数量

# 队列类，用于缓存串口接收的数据
class queue:
    def __init__(self, capacity=1024 * 4):
        self.capacity = capacity  # 队列容量
        self.size = 0  # 当前队列大小
        self.front = 0  # 队列头部索引
        self.rear = 0  # 队列尾部索引
        self.array = [0] * capacity  # 存储数据的数组

    def is_empty(self):
        return 0 == self.size  # 判断队列是否为空

    def is_full(self):
        return self.size == self.capacity  # 判断队列是否已满

    def enqueue(self, element):
        if self.is_full():
            raise Exception('queue is full')  # 队列满时抛出异常
        self.array[self.rear] = element  # 将元素加入队列尾部
        self.size += 1
        self.rear = (self.rear + 1) % self.capacity  # 更新尾部索引

    def dequeue(self):
        if self.is_empty():
            raise Exception('queue is empty')  # 队列空时抛出异常
        self.size -= 1
        self.front = (self.front + 1) % self.capacity  # 更新头部索引

    def get_front(self):
        return self.array[self.front]  # 获取队列头部元素

    def get_front_second(self):
        return self.array[((self.front + 1) % self.capacity)]  # 获取队列头部第二个元素

    def get_queue_length(self):
        return (self.rear - self.front + self.capacity) % self.capacity  # 获取队列长度

    def show_queue(self):
        for i in range(self.capacity):
            # print self.array[i],
            pass
        print(' ')

# 底盘控制类，用于与机器人底盘硬件交互
class BaseControl:
    def __init__(self):
        self.Circleloop = queue(capacity=1024*4)  # 初始化串口数据缓存队列
        # 获取ROS参数
        self.baseId = rospy.get_param('~base_id', 'base_footprint')  # 底盘坐标系ID
        self.odomId = rospy.get_param('~odom_id', 'odom')  # 里程计坐标系ID
        self.device_port = rospy.get_param('~port', '/dev/ttyUSB0')  # 串口设备端口
        self.baudrate = int(rospy.get_param('~baudrate', '115200'))  # 串口波特率
        self.odom_freq = int(rospy.get_param('~odom_freq', '50'))  # 里程计发布频率
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')  # 里程计话题
        self.battery_topic = rospy.get_param('~battery_topic', 'battery')  # 电池话题
        self.battery_freq = float(rospy.get_param('~battery_freq', '1'))  # 电池信息发布频率
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')  # 速度指令话题
        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd_topic')  # 阿克曼模型速度指令话题
        self.pub_imu = bool(rospy.get_param('~pub_imu', False))  # 是否发布IMU数据
        if self.pub_imu:
            self.imuId = rospy.get_param('~imu_id', 'imu')  # IMU坐标系ID
            self.imu_topic = rospy.get_param('~imu_topic', 'imu')  # IMU话题
            self.imu_freq = float(rospy.get_param('~imu_freq', '50'))  # IMU数据发布频率
            if self.imu_freq > 100:
                self.imu_freq = 100  # 限制IMU频率最大为100Hz
        self.pub_sonar = bool(rospy.get_param('~pub_sonar', False))  # 是否发布超声波数据
        self.sub_ackermann = bool(rospy.get_param('~sub_ackermann', False))  # 是否订阅阿克曼模型指令

        # 定义变量
        self.current_time = rospy.Time.now()  # 当前时间
        self.previous_time = self.current_time  # 上一次时间
        self.pose_x = 0.0  # 机器人x坐标
        self.pose_y = 0.0  # 机器人y坐标
        self.pose_yaw = 0.0  # 机器人偏航角
        self.serialIDLE_flag = 0  # 串口空闲标志
        self.trans_x = 0.0  # x方向速度
        self.trans_y = 0.0  # y方向速度
        self.rotat_z = 0.0  # z轴旋转速度
        self.speed = 0.0  # 阿克曼模型速度
        self.steering_angle = 0.0  # 阿克曼模型转向角
        self.sendcounter = 0  # 发送计数器
        self.ImuErrFlag = False  # IMU错误标志
        self.EncoderFlag = False  # 编码器标志
        self.BatteryFlag = False  # 电池标志
        self.OdomTimeCounter = 0  # 里程计时间计数器
        self.BatteryTimeCounter = 0  # 电池时间计数器
        self.Vx = 0  # x方向速度（编码器）
        self.Vy = 0  # y方向速度（编码器）
        self.Vyaw = 0  # 偏航角速度（编码器）
        self.Yawz = 0  # 偏航角
        self.Vvoltage = 0  # 电池电压
        self.Icurrent = 0  # 电池电流
        self.Gyro = [0, 0, 0]  # 陀螺仪数据
        self.Accel = [0, 0, 0]  # 加速度计数据
        self.Quat = [0, 0, 0, 0]  # 四元数数据
        self.Sonar = [0, 0, 0, 0]  # 超声波传感器数据
        self.movebase_firmware_version = [0, 0, 0]  # 底盘固件版本
        self.movebase_hardware_version = [0, 0, 0]  # 底盘硬件版本
        self.movebase_type = ["NanoCar", "NanoRobot", "4WD_OMNI", "4WD", "RC_ACKERMAN"]  # 底盘类型列表
        self.motor_type = ["25GA370", "37GB520", "TT48", "RS365", "RS540"]  # 电机类型列表
        self.last_cmd_vel_time = rospy.Time.now()  # 上次速度指令时间
        self.last_ackermann_cmd_time = rospy.Time.now()  # 上次阿克曼指令时间

        # 串口通信初始化
        try:
            self.serial = serial.Serial(self.device_port, self.baudrate, timeout=10)  # 打开串口
            rospy.loginfo("Opening Serial")
            try:
                if self.serial.in_waiting:
                    self.serial.readall()  # 清空串口缓冲区
            except:
                rospy.loginfo("Opening Serial Try Faild")
                pass
        except:
            rospy.logerr("Can not open Serial" + self.device_port)
            self.serial.close
            sys.exit(0)  # 串口打开失败，退出程序
        rospy.loginfo("Serial Open Succeed")

        # 根据底盘类型和配置选择订阅速度指令话题
        if (('NanoCar' in base_type) & (self.sub_ackermann == True)):
            from ackermann_msgs.msg import AckermannDriveStamped
            self.sub = rospy.Subscriber(self.ackermann_cmd_topic, AckermannDriveStamped, self.ackermannCmdCB, queue_size=20)  # 订阅阿克曼指令
        else:
            self.sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmdCB, queue_size=20)  # 订阅速度指令
        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)  # 发布里程计数据
        self.battery_pub = rospy.Publisher(self.battery_topic, BatteryState, queue_size=3)  # 发布电池数据
        if self.pub_sonar:
            if sonar_num > 0:
                self.range_pub1 = rospy.Publisher('sonar_1', Range, queue_size=3)  # 发布超声波1数据
            if sonar_num > 1:
                self.range_pub2 = rospy.Publisher('sonar_2', Range, queue_size=3)  # 发布超声波2数据
            if sonar_num > 2:
                self.range_pub3 = rospy.Publisher('sonar_3', Range, queue_size=3)  # 发布超声波3数据
            if sonar_num > 3:
                self.range_pub4 = rospy.Publisher('sonar_4', Range, queue_size=3)  # 发布超声波4数据
            if sonar_num > 0:
                self.timer_sonar = rospy.Timer(rospy.Duration(100.0/1000), self.timerSonarCB)  # 超声波数据定时器

        self.tf_broadcaster = tf.TransformBroadcaster()  # TF广播器
        self.timer_odom = rospy.Timer(rospy.Duration(1.0/self.odom_freq), self.timerOdomCB)  # 里程计定时器
        self.timer_battery = rospy.Timer(rospy.Duration(1.0/self.battery_freq), self.timerBatteryCB)  # 电池定时器
        self.timer_communication = rospy.Timer(rospy.Duration(1.0/500), self.timerCommunicationCB)  # 通信定时器

        # 如果启用IMU发布，初始化IMU发布器和定时器
        if self.pub_imu:
            self.imu_pub = rospy.Publisher(self.imu_topic, Imu, queue_size=10)  # 发布IMU数据
            self.timer_imu = rospy.Timer(rospy.Duration(1.0/self.imu_freq), self.timerIMUCB)  # IMU数据定时器
        self.getVersion()  # 获取底盘版本信息
        # 底盘IMU初始化需要约2秒，期间系统会被阻塞
        while self.movebase_hardware_version[0] == 0:
            pass
        if self.movebase_hardware_version[0] < 2:
            time.sleep(2.0)  # 旧版本硬件需要等待2秒
        self.getSN()  # 获取底盘序列号
        time.sleep(0.01)
        self.getInfo()  # 获取底盘信息

    # CRC-8校验计算
    def crc_1byte(self, data):
        crc_1byte = 0
        for i in range(0, 8):
            if ((crc_1byte ^ data) & 0x01):
                crc_1byte ^= 0x18
                crc_1byte >>= 1
                crc_1byte |= 0x80
            else:
                crc_1byte >>= 1
            data >>= 1
        return crc_1byte

    def crc_byte(self, data, length):
        ret = 0
        for i in range(length):
            ret = self.crc_1byte(ret ^ data[i])  # 对数据逐字节进行CRC校验
        return ret

    # 速度指令回调函数，处理Twist消息并发送至底盘
    def cmdCB(self, data):
        self.trans_x = data.linear.x  # x方向速度
        self.trans_y = data.linear.y  # y方向速度
        self.rotat_z = data.angular.z  # z轴角速度
        self.last_cmd_vel_time = rospy.Time.now()  # 更新指令时间
        outputdata = [0x5a, 0x0c, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # 速度指令数据包
        outputdata[4] = (int(self.trans_x * 1000.0) >> 8) & 0xff  # x速度高字节
        outputdata[5] = int(self.trans_x * 1000.0) & 0xff  # x速度低字节
        outputdata[6] = (int(self.trans_y * 1000.0) >> 8) & 0xff  # y速度高字节
        outputdata[7] = int(self.trans_y * 1000.0) & 0xff  # y速度低字节
        outputdata[8] = (int(self.rotat_z * 1000.0) >> 8) & 0xff  # z角速度高字节
        outputdata[9] = int(self.rotat_z * 1000.0) & 0xff  # z角速度低字节
        crc_8 = self.crc_byte(outputdata, len(outputdata) - 1)  # 计算CRC校验
        outputdata[11] = crc_8  # 添加CRC校验字节
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送速度指令
        except:
            rospy.logerr("Vel Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0

    # 阿克曼模型速度指令回调函数，处理AckermannDriveStamped消息
    def ackermannCmdCB(self, data):
        self.speed = data.drive.speed  # 速度
        self.steering_angle = data.drive.steering_angle  # 转向角
        self.last_ackermann_cmd_time = rospy.Time.now()  # 更新指令时间
        outputdata = [0x5a, 0x0c, 0x01, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # 阿克曼指令数据包
        outputdata[4] = (int(self.speed * 1000.0) >> 8) & 0xff  # 速度高字节
        outputdata[5] = int(self.speed * 1000.0) & 0xff  # 速度低字节
        outputdata[8] = (int(self.steering_angle * 1000.0) >> 8) & 0xff  # 转向角高字节
        outputdata[9] = int(self.steering_angle * 1000.0) & 0xff  # 转向角低字节
        crc_8 = self.crc_byte(outputdata, len(outputdata) - 1)  # 计算CRC校验
        outputdata[11] = crc_8  # 添加CRC校验字节
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 4
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送阿克曼指令
        except:
            rospy.logerr("Vel Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0

    # 获取底盘硬件和固件版本
    def getVersion(self):
        outputdata = [0x5a, 0x06, 0x01, 0xf1, 0x00, 0xd7]  # 版本信息请求数据包
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送版本请求
        except:
            rospy.logerr("Get Version Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0

    # 获取底盘序列号
    def getSN(self):
        outputdata = [0x5a, 0x06, 0x01, 0xf3, 0x00, 0x46]  # 序列号请求数据包
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送序列号请求
        except:
            rospy.logerr("Get SN Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0

    # 获取底盘信息
    def getInfo(self):
        outputdata = [0x5a, 0x06, 0x01, 0x21, 0x00, 0x8f]  # 底盘信息请求数据包
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送信息请求
        except:
            rospy.logerr("Get info Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0

    # 里程计定时器回调函数，获取速度和IMU信息并转换为里程计话题
    def timerOdomCB(self, event):
        # 根据固件版本选择不同的速度数据请求命令
        if self.movebase_firmware_version[1] == 0:
            outputdata = [0x5a, 0x06, 0x01, 0x09, 0x00, 0x38]  # 旧版固件命令
        else:
            outputdata = [0x5a, 0x06, 0x01, 0x11, 0x00, 0xa2]  # 新版固件命令
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 1
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送速度数据请求
        except:
            rospy.logerr("Odom Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0

        # 计算里程计数据
        Vx = float(ctypes.c_int16(self.Vx).value / 1000.0)  # x方向速度（m/s）
        Vy = float(ctypes.c_int16(self.Vy).value / 1000.0)  # y方向速度（m/s）
        Vyaw = float(ctypes.c_int16(self.Vyaw).value / 1000.0)  # 偏航角速度（rad/s）
        self.pose_yaw = float(ctypes.c_int16(self.Yawz).value / 100.0)  # 偏航角（度）
        self.pose_yaw = self.pose_yaw * math.pi / 180.0  # 转换为弧度

        self.current_time = rospy.Time.now()  # 当前时间
        dt = (self.current_time - self.previous_time).to_sec()  # 时间差
        self.previous_time = self.current_time
        # 更新机器人位置
        self.pose_x = self.pose_x + Vx * (math.cos(self.pose_yaw)) * dt - Vy * (math.sin(self.pose_yaw)) * dt
        self.pose_y = self.pose_y + Vx * (math.sin(self.pose_yaw)) * dt + Vy * (math.cos(self.pose_yaw)) * dt

        pose_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose_yaw)  # 偏航角转换为四元数
        msg = Odometry()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.odomId
        msg.child_frame_id = self.baseId
        msg.pose.pose.position.x = self.pose_x
        msg.pose.pose.position.y = self.pose_y
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation.x = pose_quat[0]
        msg.pose.pose.orientation.y = pose_quat[1]
        msg.pose.pose.orientation.z = pose_quat[2]
        msg.pose.pose.orientation.w = pose_quat[3]
        msg.twist.twist.linear.x = Vx
        msg.twist.twist.linear.y = Vy
        msg.twist.twist.angular.z = Vyaw
        self.pub.publish(msg)  # 发布里程计数据
        self.tf_broadcaster.sendTransform((self.pose_x, self.pose_y, 0.0), pose_quat, self.current_time, self.baseId, self.odomId)  # 广播TF变换

    # 电池定时器回调函数，获取电池信息
    def timerBatteryCB(self, event):
        outputdata = [0x5a, 0x06, 0x01, 0x07, 0x00, 0xe4]  # 电池信息请求数据包
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送电池信息请求
        except:
            rospy.logerr("Battery Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0
        msg = BatteryState()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.baseId
        msg.voltage = float(self.Vvoltage / 1000.0)  # 电池电压（V）
        msg.current = float(self.Icurrent / 1000.0)  # 电池电流（A）
        self.battery_pub.publish(msg)  # 发布电池数据

    # 超声波定时器回调函数，获取超声波传感器数据
    def timerSonarCB(self, event):
        outputdata = [0x5a, 0x06, 0x01, 0x19, 0x00, 0xd4]  # 超声波数据请求数据包
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送超声波数据请求
        except:
            rospy.logerr("Sonar Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0
        msg = Range()
        msg.header.stamp = self.current_time
        msg.field_of_view = 0.26  # 超声波视场角约15度
        msg.max_range = 2.5  # 最大测量距离（m）
        msg.min_range = 0.01  # 最小测量距离（m）
        # 超声波传感器1
        msg.header.frame_id = 'Sonar_1'
        if self.Sonar[0] == 0xff:
            msg.range = float('inf')  # 无效数据表示无穷远
        else:
            msg.range = self.Sonar[0] / 100.0  # 转换为米
        self.range_pub1.publish(msg)  # 发布超声波1数据

        # 根据底盘类型设置超声波传感器1的TF变换
        if 'NanoRobot' in base_type:
            self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.current_time, 'Sonar_1', self.baseId)
        elif 'NanoCar' in base_type:
            self.tf_broadcaster.sendTransform((0.18, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.current_time, 'Sonar_1', self.baseId)
        elif '4WD' in base_type:
            self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.current_time, 'Sonar_1', self.baseId)
        elif 'Race182' in base_type:
            self.tf_broadcaster.sendTransform((0.18, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.current_time, 'Sonar_1', self.baseId)
        elif 'NanoOmni' in base_type:
            self.tf_broadcaster.sendTransform((0.11, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.current_time, 'Sonar_1', self.baseId)
        else:
            pass

        # 超声波传感器2
        if sonar_num > 1:
            if self.Sonar[1] == 0xff:
                msg.range = float('inf')
            else:
                msg.range = self.Sonar[1] / 100.0
            msg.header.frame_id = 'Sonar_2'
            self.range_pub2.publish(msg)  # 发布超声波2数据
            if 'NanoRobot' in base_type:
                self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, -1.0, 0), self.current_time, 'Sonar_2', self.baseId)
            elif 'NanoCar' in base_type:
                self.tf_broadcaster.sendTransform((-0.035, 0.0, 0.0), (0.0, 0.0, -1.0, 0), self.current_time, 'Sonar_2', self.baseId)
            elif '4WD' in base_type:
                self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, -1.0, 0), self.current_time, 'Sonar_2', self.baseId)
            elif 'Race182' in base_type:
                self.tf_broadcaster.sendTransform((-0.08, 0.0, 0.0), (0.0, 0.0, -1.0, 0), self.current_time, 'Sonar_2', self.baseId)
            elif 'NanoOmni' in base_type:
                self.tf_broadcaster.sendTransform((-0.11, 0.0, 0.0), (0.0, 0.0, -1.0, 0.0), self.current_time, 'Sonar_2', self.baseId)
            else:
                pass
        # 超声波传感器3
        if sonar_num > 2:
            msg.header.frame_id = 'Sonar_3'
            if self.Sonar[2] == 0xff:
                msg.range = float('inf')
            else:
                msg.range = self.Sonar[2] / 100.0
            self.range_pub3.publish(msg)  # 发布超声波3数据
            if 'Race182' in base_type:
                self.tf_broadcaster.sendTransform((0.0, 0.06, 0.0), (0.0, 0.0, 0.707, 0.707), self.current_time, 'Sonar_3', self.baseId)
            elif 'NanoOmni' in base_type:
                self.tf_broadcaster.sendTransform((0.0, 0.07, 0.0), (0.0, 0.0, 0.707, 0.707), self.current_time, 'Sonar_3', self.baseId)
        # 超声波传感器4
        if sonar_num > 3:
            if self.Sonar[3] == 0xff:
                msg.range = float('inf')
            else:
                msg.range = self.Sonar[3] / 100.0
            msg.header.frame_id = 'Sonar_4'
            self.range_pub4.publish(msg)  # 发布超声波4数据
            if 'Race182' in base_type:
                self.tf_broadcaster.sendTransform((0.0, -0.06, 0.0), (0.0, 0.0, -0.707, 0.707), self.current_time, 'Sonar_4', self.baseId)
            elif 'NanoOmni' in base_type:
                self.tf_broadcaster.sendTransform((0.0, -0.07, 0.0), (0.0, 0.0, -0.707, 0.707), self.current_time, 'Sonar_4', self.baseId)

    # IMU定时器回调函数，获取IMU原始数据
    def timerIMUCB(self, event):
        outputdata = [0x5a, 0x06, 0x01, 0x13, 0x00, 0x33]  # IMU数据请求数据包
        while self.serialIDLE_flag:
            time.sleep(0.01)  # 等待串口空闲
        self.serialIDLE_flag = 3
        try:
            while self.serial.out_waiting:
                pass  # 等待串口输出缓冲区清空
            self.serial.write(outputdata)  # 发送IMU数据请求
        except:
            rospy.logerr("Imu Command Send Faild")  # 发送失败日志
        self.serialIDLE_flag = 0
        msg = Imu()
        msg.header.stamp = self.current_time
        msg.header.frame_id = self.imuId
        # 角速度（rad/s）
        msg.angular_velocity.x = float(ctypes.c_int32(self.Gyro[0]).value / 100000.0)
        msg.angular_velocity.y = float(ctypes.c_int32(self.Gyro[1]).value / 100000.0)
        msg.angular_velocity.z = float(ctypes.c_int32(self.Gyro[2]).value / 100000.0)
        # 线性加速度（m/s^2）
        msg.linear_acceleration.x = float(ctypes.c_int32(self.Accel[0]).value / 100000.0)
        msg.linear_acceleration.y = float(ctypes.c_int32(self.Accel[1]).value / 100000.0)
        msg.linear_acceleration.z = float(ctypes.c_int32(self.Accel[2]).value / 100000.0)
        # 四元数
        msg.orientation.w = float(ctypes.c_int16(self.Quat[0]).value / 10000.0)
        msg.orientation.x = float(ctypes.c_int16(self.Quat[1]).value / 10000.0)
        msg.orientation.y = float(ctypes.c_int16(self.Quat[2]).value / 10000.0)
        msg.orientation.z = float(ctypes.c_int16(self.Quat[3]).value / 10000.0)
        self.imu_pub.publish(msg)  # 发布IMU数据
        # 根据底盘类型设置IMU的TF变换
        if 'NanoRobot' in base_type:
            self.tf_broadcaster.sendTransform((-0.062, -0.007, 0.08), (0.0, 0.0, 0.0, 1.0), self.current_time, self.imuId, self.baseId)
        elif 'NanoCar' in base_type:
            self.tf_broadcaster.sendTransform((0.0, 0.0, 0.09), (0.0, 0.0, 0.0, 1.0), self.current_time, self.imuId, self.baseId)
        elif '4WD' in base_type:
            self.tf_broadcaster.sendTransform((-0.065, 0.0167, 0.02), (0.0, 0.0, 0.0, 1.0), self.current_time, self.imuId, self.baseId)
        else:
            self.tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.current_time, self.imuId, self.baseId)

    # 通信定时器回调函数，处理接收到的串口数据
    def timerCommunicationCB(self, event):
        length = self.serial.in_waiting
        if length:
            reading = self.serial.read_all()  # 读取所有串口数据
            if len(reading) != 0:
                for i in range(0, len(reading)):
                    data = reading[i]
                    try:
                        self.Circleloop.enqueue(data)  # 将数据加入队列
                    except:
                        pass
        else:
            pass
        if not self.Circleloop.is_empty():
            data = self.Circleloop.get_front()
            if data == 0x5a:  # 检查数据包头
                length = self.Circleloop.get_front_second()
                if length > 1 and self.Circleloop.get_front_second() <= self.Circleloop.get_queue_length():
                    databuf = []
                    for i in range(length):
                        databuf.append(self.Circleloop.get_front())  # 读取数据包
                        self.Circleloop.dequeue()
                    if databuf[length - 1] == self.crc_byte(databuf, length - 1):  # 校验CRC
                        pass
                    else:
                        return
                    # 解析接收到的数据
                    if databuf[3] == 0x04:
                        self.Vx = databuf[4] * 256 + databuf[5]  # x方向速度
                        self.Vy = databuf[6] * 256 + databuf[7]  # y方向速度
                        self.Vyaw = databuf[8] * 256 + databuf[9]  # 偏航角速度
                    elif databuf[3] == 0x06:
                        self.Yawz = databuf[8] * 256 + databuf[9]  # 偏航角
                    elif databuf[3] == 0x08:
                        self.Vvoltage = databuf[4] * 256 + databuf[5]  # 电池电压
                        self.Icurrent = databuf[6] * 256 + databuf[7]  # 电池电流
                    elif databuf[3] == 0x0a:
                        self.Vx = databuf[4] * 256 + databuf[5]  # x方向速度
                        self.Yawz = databuf[6] * 256 + databuf[7]  # 偏航角
                        self.Vyaw = databuf[8] * 256 + databuf[9]  # 偏航角速度
                    elif databuf[3] == 0x12:
                        self.Vx = databuf[4] * 256 + databuf[5]  # x方向速度
                        self.Vy = databuf[6] * 256 + databuf[7]  # y方向速度
                        self.Yawz = databuf[8] * 256 + databuf[9]  # 偏航角
                        self.Vyaw = databuf[10] * 256 + databuf[11]  # 偏航角速度
                    elif databuf[3] == 0x14:
                        self.Gyro[0] = int(((databuf[4] & 0xff) << 24) | ((databuf[5] & 0xff) << 16) | ((databuf[6] & 0xff) << 8) | (databuf[7] & 0xff))  # 陀螺仪x
                        self.Gyro[1] = int(((databuf[8] & 0xff) << 24) | ((databuf[9] & 0xff) << 16) | ((databuf[10] & 0xff) << 8) | (databuf[11] & 0xff))  # 陀螺仪y
                        self.Gyro[2] = int(((databuf[12] & 0xff) << 24) | ((databuf[13] & 0xff) << 16) | ((databuf[14] & 0xff) << 8) | (databuf[15] & 0xff))  # 陀螺仪z
                        self.Accel[0] = int(((databuf[16] & 0xff) << 24) | ((databuf[17] & 0xff) << 16) | ((databuf[18] & 0xff) << 8) | (databuf[19] & 0xff))  # 加速度x
                        self.Accel[1] = int(((databuf[20] & 0xff) << 24) | ((databuf[21] & 0xff) << 16) | ((databuf[22] & 0xff) << 8) | (databuf[23] & 0xff))  # 加速度y
                        self.Accel[2] = int(((databuf[24] & 0xff) << 24) | ((databuf[25] & 0xff) << 16) | ((databuf[26] & 0xff) << 8) | (databuf[27] & 0xff))  # 加速度z
                        self.Quat[0] = int((databuf[28] & 0xff) << 8 | databuf[29])  # 四元数w
                        self.Quat[1] = int((databuf[30] & 0xff) << 8 | databuf[31])  # 四元数x
                        self.Quat[2] = int((databuf[32] & 0xff) << 8 | databuf[33])  # 四元数y
                        self.Quat[3] = int((databuf[34] & 0xff) << 8 | databuf[35])  # 四元数z
                    elif databuf[3] == 0x1a:
                        self.Sonar[0] = databuf[4]  # 超声波传感器1
                        self.Sonar[1] = databuf[5]  # 超声波传感器2
                        self.Sonar[2] = databuf[6]  # 超声波传感器3
                        self.Sonar[3] = databuf[7]  # 超声波传感器4
                    elif databuf[3] == 0xf2:
                        self.movebase_hardware_version[0] = databuf[4]  # 硬件版本主版本号
                        self.movebase_hardware_version[1] = databuf[5]  # 硬件版本次版本号
                        self.movebase_hardware_version[2] = databuf[6]  # 硬件版本修订号
                        self.movebase_firmware_version[0] = databuf[7]  # 固件版本主版本号
                        self.movebase_firmware_version[1] = databuf[8]  # 固件版本次版本号
                        self.movebase_firmware_version[2] = databuf[9]  # 固件版本修订号
                        version_string = "Move Base Hardware Ver %d.%d.%d,Firmware Ver %d.%d.%d" % (
                            self.movebase_hardware_version[0], self.movebase_hardware_version[1], self.movebase_hardware_version[2],
                            self.movebase_firmware_version[0], self.movebase_firmware_version[1], self.movebase_firmware_version[2])
                        rospy.loginfo(version_string)  # 打印版本信息
                    elif databuf[3] == 0xf4:
                        sn_string = "SN:"
                        for i in range(4, 16):
                            sn_string = "%s%02x" % (sn_string, databuf[i])  # 拼接序列号
                        rospy.loginfo(sn_string)  # 打印序列号
                    elif databuf[3] == 0x22:
                        fRatio = float(databuf[6] << 8 | databuf[7]) / 10  # 减速比
                        fDiameter = float(databuf[8] << 8 | databuf[9]) / 10  # 轮径
                        info_string = "Type:%s Motor:%s Ratio:%.01f WheelDiameter:%.01f" % (
                            self.movebase_type[databuf[4] - 1], self.motor_type[databuf[5] - 1], fRatio, fDiameter)
                        rospy.loginfo(info_string)  # 打印底盘信息
                    else:
                        pass
                else:
                    pass
            else:
                self.Circleloop.dequeue()  # 移除无效数据
        else:
            pass

# 主函数
if __name__ == "__main__":
    try:
        rospy.init_node('base_control', anonymous=True)  # 初始化ROS节点
        if base_type != None:
            rospy.loginfo('%s base control ...' % base_type)  # 打印底盘类型
        else:
            rospy.loginfo('base control ...')
            rospy.logerr('PLEASE SET BASE_TYPE ENV FIRST')  # 提示设置底盘类型环境变量
        bc = BaseControl()  # 创建底盘控制对象
        rospy.spin()  # 进入ROS事件循环
    except KeyboardInterrupt:
        bc.serial.close  # 关闭串口
        print("Shutting down")  # 程序退出提示