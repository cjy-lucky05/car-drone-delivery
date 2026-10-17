# NVILIDAR ROS DRIVER

## How to [install ROS](http://wiki.ros.org/cn/ROS/Installation)

[ubuntu](http://wiki.ros.org/cn/Installation/Ubuntu)

[windows](http://wiki.ros.org/Installation/Windows)

## How to Create a ROS workspace

[Create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

you also can with this:

    1)  $mkdir -p ~/nvilidar_ros_ws/src
        $cd ~/nvilidar_ros_ws/src
    2)  $cd..
    3)  $catkin_make
    4)  $source devel/setup.bash
    5)  echo $ROS_PACKAGE_PATH
        /home/user/nvilidar_ws/src:/opt/ros/kinetic/share


## How to build NVILiDAR ROS Package
### 1.Get the ros code
    1) Clone this project to your catkin's workspace src folder
    	(1). git clone https://github.com/nvilidar/nvilidar_ros.git  
             or
             git clone https://gitee.com/nvilidar/nvilidar_ros.git
    	(2). git chectout master

    2) download the code from our webset,  http://www.nvistar.com/?jishuzhichi/xiazaizhongxin
    
### 2.Copy the ros code
    1) Copy the ros source file to the "~/nvilidar_ros_ws/src"
    2) Running "catkin_make" to build nvilidar_node and nvilidar_client
### 3.Serialport configuration
    1) if you want to use the unchanging device name,Create the name "/dev/nvilidar" to rename serialport
        --$ roscd nvilidar_ros/startup
        --$ sudo chmod 777 ./*
        --$ sudo sh initenv.sh
    2) if you use the lidar device name,you must give the permissions to user.
        ---whoami
       get the user name.link ubuntu.
        ---sudo usermod -a -G dialout ubuntu
       ubuntu is the user name.
        ---sudo reboot   

## ROS Parameter Configuration
### 1. Lidar Support
    current ros support 2 types of Lidar,include VP300 and VP350.ros can automatic lidar model recognition.

    if you want to use serialport,

    The VP300 is a USB interface lidar, supporting various baud rates, bps is recommended.
    VP350 is a serial interface lidar, supporting only 512000bps.


### 2. Choice the ROS Communication interface

    	(1). if you want use the serialport,you neet to change the code from "nvilidar_node.cpp",change the code like this:

```cpp
//choice use serialport or socket 
#if 1
    nvilidar::LidarProcess laser(USE_SERIALPORT,cfg.serialport_name,cfg.serialport_baud);
#else 
    nvilidar::LidarProcess laser(USE_SOCKET,cfg.ip_addr, cfg.lidar_udp_port);
#endif 
```
the 2 parameters represent the serialport name and serialport baudrate.

        (2). if you want to use the udp socket,you neet to change the code 
        from "nvilidar_node.cpp",change the code like this:

```cpp
//choice use serialport or socket 
#if 0
    nvilidar::LidarProcess laser(USE_SERIALPORT,cfg.serialport_name,cfg.serialport_baud);
#else 
    nvilidar::LidarProcess laser(USE_SOCKET,cfg.ip_addr, cfg.lidar_udp_port);
#endif 
```

the 2 parameters represent IP address and port number. 

## Interface function definition
### 1. bool LidarProcess::LidarInitialialize()
    Initialize the lidar, including opening the serial/socket interface and synchronizing the radar with the SDK parameter information
	if initial fail return false.
### 2. bool LidarProcess::LidarTurnOn()
	Turn on lidar scanning so that it can output point cloud data.
### 3. bool LidarProcess::LidarSamplingProcess(LidarScan &scan, uint32_t timeout)
	Real-time radar data output interface.
	The LidarScan variables are described as follows:
 
|  value   | Element | define  |
|  :----:  | :----:  | :----:  |
|  stamp   | none    |  lidar stamps, unit ns|
|  config  | min_angle   | lidar angle min value, 0~2*PI,Unit Radian|
|          | max_angle   | lidar angle max value, 0~2*PI,Unit Radian|
|          | angle_increment   | angular interval between 2 points, 0~2PI|
|          | scan_time   | time interval of 2 turns of data|
|          | min_range   | Distance measurement min, unit m|
|          | max_range   | Distance measurement max, unit m|
|  points  | angle       | lidar angle,0~2PI|
|  | range       | lidar distance, unit m|
|  | intensity       | lidar intensity,it is aviliable when sensitive is true|
### 4. bool LidarProcess::LidarTurnOff()
	lidar turn off the scanning data 
### 5. void LidarProcess::LidarCloseHandle()
	lidar close serialport/socket 

## How to Run NVILIDAR ROS Package
### 1. Run NVILIDAR node and view in the rviz
------------------------------------------------------------
	roslaunch nvilidar_ros lidar_view.launch

### 2. Run NVILIDAR node and view using test application
------------------------------------------------------------
	roslaunch nvilidar_ros lidar.launch

	rosrun nvilidar_ros nvilidar_client

## NVILIDAR ROS Parameter
|  value   |  information  |
|  :----:    | :----:  |
| serialport_baud  | if use serialport,the lidar's serialport |
| serialport_name  | if use serialport,the lidar's port name |
| ip_addr  | if use udp socket,the lidar's ip addr,default:192.168.1.200 |
| lidar_udp_port  | if use udp socket,the lidar's udp port,default:8100 |
| config_tcp_port  | if use udp socket,config the net converter's para,default:8200 |
| frame_id  | it is useful in ros,lidar ros frame id |
| resolution_fixed  | Rotate one circle fixed number of points,it is 'true' in ros,default |
| auto_reconnect  | lidar auto connect,if it is disconnet in case |
| reversion  | lidar's point revert|
| inverted  | lidar's point invert|
| angle_max  | lidar angle max value,max:180.0°|
| angle_max  | lidar angle min value,min:-180.0°|
| range_max  | lidar's max measure distance,default:64.0 meters|
| range_min  | lidar's min measure distance,default:0.0 meters|
| aim_speed  | lidar's run speed,default:10.0 Hz|
| sampling_rate  | lidar's sampling rate,default:10.0 K points in 1 second|
| sensitive  | lidar's data with sensitive,default:false|
| tailing_level  | lidar's tailing level,The smaller the value, the stronger the filtering,default:6|
| angle_offset_change_flag  | angle offset enable to set,default:false|
| angle_offset  | angle offset,default:0.0|
| adp_change_flag  | change apd value,don't change it if nessesary,default:false|
| adp_value  | change apd value,if the 'apd_change_flag' is true,it is valid,default:500|
| ignore_array_string  | if you want to filter some point's you can change it,it is anti-clockwise for the lidar.eg. you can set the value "30,60,90,120",you can remove the 30°~60° and 90°~120° points in the view|
| filter_jump_enable | filter some jump point,default:true |
| filter_jump_value_min | filter some jump point,min value,Not recommended to modify |
| filter_jump_value_max | filter some jump point,max value,Not recommended to modify |