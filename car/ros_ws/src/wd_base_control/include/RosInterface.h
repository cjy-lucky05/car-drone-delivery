#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include "types.h"
#include "RobotState.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <memory>
#include <nlohmann/json.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <vector>
#include <random>
#include <iostream>
#include <cmath>

using nlohmann::json;

class RosInterface
{
public:
    RosInterface(std::shared_ptr<RobotState> robot_state);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void markersCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
    void droneMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr &msg);
    void patrolCallback(const std_msgs::Bool::ConstPtr &msg);
    void updateObjectPoseList();
    void getGlobalToMapOriginMat();
    void getRobotToMapOriginMat();
    void publishGoal();                 
    void publishGoal(const Pose &goal); 
    void publishCmdVel(double linear, double angular);
    void publishGimbalCmd(int pwm_x, int pwm_y);
    void setPatrolPoints(const std::vector<Pose> &points);
    void startPatrol(size_t index);
    void cancelGoal();
    bool isGoalReached();
    void randomMove();
    void fireOnce();
    nav_msgs::OccupancyGrid::ConstPtr getMapMsg() const { return map_msg_; }
    std::vector<ObjectPose> getObjectPoseList() const { return object_pose_list_; }
    Pose getRobotInMapOrigin();
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);

public:
    void moveBaseResultCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);

    std::shared_ptr<RobotState> robot_state_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string map_frame_;
    std::string robot_frame_;
    ros::Subscriber map_sub_;
    ros::Subscriber object_markers_sub_;
    ros::Subscriber drone_markers_sub_;
    ros::Subscriber patrol_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher attack_pub_;
    ros::Publisher cancel_pub_;
    ros::Publisher serial_pub_;
    nav_msgs::OccupancyGrid::ConstPtr map_msg_;
    Eigen::Matrix4d map_origin_in_global_mat_;
    Eigen::Matrix4d global_in_map_origin_mat_;
    Eigen::Matrix4d robot_in_map_origin_mat_;
    Eigen::Matrix4d goal_in_global_mat_;
    geometry_msgs::PoseStamped goal_pose_msg_;
    visualization_msgs::MarkerArray::ConstPtr object_markers_msg_;
    visualization_msgs::MarkerArray::ConstPtr drone_markers_msg_;
    std::vector<Pose> patrol_points_;
    size_t current_patrol_index_;
    bool patrol_active_;
    std::vector<ObjectPose> object_pose_list_;
    std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_client_;
    bool goal_reached_;

private:
    ros::Subscriber image_sub_;
    cv::dnn::Net network_;
    std::vector<std::string> CLASSES_;
    std::vector<cv::Scalar> COLORS_;
    double K_rotation_;
    double K_velocity_;
    double max_velocity_;
};

#endif // ROS_INTERFACE_H