#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

class ROSInterface {
public:
    ROSInterface();
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void markersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void droneMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void patrolCallback(const std_msgs::Bool::ConstPtr& msg);
    void updateObjectPoseList();
    void getGlobalToMapOriginMat();
    void getRobotToMapOriginMat();
    void publishGoal();
    void publishCmdVel(double linear, double angular);
    void setPatrolPoints(const std::vector<Pose>& points);
    void startPatrol();
    nav_msgs::OccupancyGrid::ConstPtr getMapMsg() const { return map_msg_; }
    std::vector<ObjectPose> getObjectPoseList() const { return object_pose_list_; }
    Pose getRobotInMapOrigin();

public:
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
};

#endif // ROS_INTERFACE_H

#include "ros_interface.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

ROSInterface::ROSInterface() : tf_listener_(tf_buffer_),
    map_frame_(ros::NodeHandle().param<std::string>("/ros_interface/map_frame", "map")),
    robot_frame_(ros::NodeHandle().param<std::string>("/ros_interface/robot_frame", "base_link")),
    map_origin_in_global_mat_(Eigen::Matrix4d::Identity()),
    global_in_map_origin_mat_(Eigen::Matrix4d::Identity()),
    robot_in_map_origin_mat_(Eigen::Matrix4d::Identity()),
    goal_in_global_mat_(Eigen::Matrix4d::Identity()),
    current_patrol_index_(0), patrol_active_(false) {
    ros::NodeHandle nh;
    map_sub_ = nh.subscribe("map", 10, &ROSInterface::mapCallback, this);
    object_markers_sub_ = nh.subscribe("object_markers", 10, &ROSInterface::markersCallback, this);
    drone_markers_sub_ = nh.subscribe("drone_markers", 10, &ROSInterface::droneMarkersCallback, this);
    patrol_sub_ = nh.subscribe("/patrol_cmd", 10, &ROSInterface::patrolCallback, this);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    attack_pub_ = nh.advertise<std_msgs::Bool>("/attack_cmd", 10);
}

void ROSInterface::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_msg_ = msg;
    ROS_INFO("Map received: width=%u, height=%u, resolution=%f, origin_x=%f, origin_y=%f",
             msg->info.width, msg->info.height, msg->info.resolution,
             msg->info.origin.position.x, msg->info.origin.position.y);
}

void ROSInterface::markersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    object_markers_msg_ = msg;
    updateObjectPoseList();
}

void ROSInterface::droneMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    drone_markers_msg_ = msg;
    updateObjectPoseList();
}

void ROSInterface::updateObjectPoseList() {
    object_pose_list_.clear();
    if (object_markers_msg_) {
        for (const auto& marker : object_markers_msg_->markers) {
            ObjectPose pose;
            pose.name = marker.ns + "_" + std::to_string(marker.id);
            pose.position.x = marker.pose.position.x;
            pose.position.y = marker.pose.position.y;
            pose.position.z = marker.pose.position.z;
            object_pose_list_.push_back(pose);
        }
    }
    if (drone_markers_msg_) {
        for (const auto& marker : drone_markers_msg_->markers) {
            ObjectPose pose;
            pose.name = marker.ns + "_" + std::to_string(marker.id);
            pose.position.x = marker.pose.position.x;
            pose.position.y = marker.pose.position.y;
            pose.position.z = marker.pose.position.z;
            object_pose_list_.push_back(pose);
        }
    }
}

void ROSInterface::getGlobalToMapOriginMat() {
    try {
        tf2::Transform transform;
        tf2::fromMsg(map_msg_->info.origin, transform);
        Eigen::Matrix3d rot_mat;
        rot_mat << transform.getBasis()[0][0], transform.getBasis()[0][1], transform.getBasis()[0][2],
                   transform.getBasis()[1][0], transform.getBasis()[1][1], transform.getBasis()[1][2],
                   transform.getBasis()[2][0], transform.getBasis()[2][1], transform.getBasis()[2][2];
        map_origin_in_global_mat_.block<3, 3>(0, 0) = rot_mat;
        map_origin_in_global_mat_.block<3, 1>(0, 3) = Eigen::Vector3d(
            map_msg_->info.origin.position.x,
            map_msg_->info.origin.position.y,
            map_msg_->info.origin.position.z);
        global_in_map_origin_mat_.block<3, 3>(0, 0) = rot_mat.transpose();
        global_in_map_origin_mat_.block<3, 1>(0, 3) = -rot_mat.transpose() * map_origin_in_global_mat_.block<3, 1>(0, 3);
    } catch (const std::exception& e) {
        ROS_WARN("Failed to compute global to map origin transform: %s", e.what());
    }
}

void ROSInterface::getRobotToMapOriginMat() {
    try {
        geometry_msgs::TransformStamped robot_in_map = tf_buffer_.lookupTransform(
            map_frame_, robot_frame_, ros::Time(0), ros::Duration(1.0));
        tf2::Transform transform;
        tf2::fromMsg(robot_in_map.transform, transform);
        Eigen::Matrix3d rot_mat;
        rot_mat << transform.getBasis()[0][0], transform.getBasis()[0][1], transform.getBasis()[0][2],
                   transform.getBasis()[1][0], transform.getBasis()[1][1], transform.getBasis()[1][2],
                   transform.getBasis()[2][0], transform.getBasis()[2][1], transform.getBasis()[2][2];
        robot_in_map_origin_mat_.block<3, 3>(0, 0) = rot_mat;
        robot_in_map_origin_mat_.block<3, 1>(0, 3) = Eigen::Vector3d(
            robot_in_map.transform.translation.x,
            robot_in_map.transform.translation.y,
            robot_in_map.transform.translation.z);
        // ROS_INFO("Robot pose in map: x=%f, y=%f, z=%f",
        //          robot_in_map_origin_mat_(0, 3), robot_in_map_origin_mat_(1, 3), robot_in_map_origin_mat_(2, 3));
    } catch (const tf2::LookupException& e) {
        ROS_WARN("Failed to get map to base_link transform: %s", e.what());
    }
}

void ROSInterface::publishGoal() {
    goal_pose_msg_.header.frame_id = "map";
    goal_pose_msg_.header.stamp = ros::Time::now();
    goal_pose_msg_.pose.position.x = goal_in_global_mat_(0, 3);
    goal_pose_msg_.pose.position.y = goal_in_global_mat_(1, 3);
    goal_pose_msg_.pose.position.z = goal_in_global_mat_(2, 3);
    Eigen::Matrix3d rot = goal_in_global_mat_.block<3, 3>(0, 0);
    tf2::Matrix3x3 tf_rot(rot(0, 0), rot(0, 1), rot(0, 2),
                          rot(1, 0), rot(1, 1), rot(1, 2),
                          rot(2, 0), rot(2, 1), rot(2, 2));
    tf2::Quaternion q;
    tf_rot.getRotation(q);
    goal_pose_msg_.pose.orientation = tf2::toMsg(q);
    goal_pub_.publish(goal_pose_msg_);
    ROS_INFO("Published goal: position=(%f, %f, %f), orientation=(%f, %f, %f, %f)",
             goal_pose_msg_.pose.position.x, goal_pose_msg_.pose.position.y, goal_pose_msg_.pose.position.z,
             goal_pose_msg_.pose.orientation.x, goal_pose_msg_.pose.orientation.y,
             goal_pose_msg_.pose.orientation.z, goal_pose_msg_.pose.orientation.w);
}

void ROSInterface::publishCmdVel(double linear, double angular) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;
    cmd_vel_pub_.publish(cmd_vel);
    ROS_INFO("Published cmd_vel: linear=%f, angular=%f", linear, angular);
}

void ROSInterface::setPatrolPoints(const std::vector<Pose>& points) {
    patrol_points_ = points;
    current_patrol_index_ = 0;
    ROS_INFO("Updated patrol points: %lu points", points.size());
}

void ROSInterface::patrolCallback(const std_msgs::Bool::ConstPtr& msg) {
    patrol_active_ = msg->data;
    if (patrol_active_ && !patrol_points_.empty()) {
        current_patrol_index_ = 0;
        startPatrol();
    } else if (!patrol_active_) {
        ROS_INFO("Patrol stopped");
    }
}

void ROSInterface::startPatrol() {
    if (!patrol_active_ || patrol_points_.empty()) {
        ROS_WARN("Patrol not active or no patrol points, skipping");
        return;
    }
    const auto& goal = patrol_points_[current_patrol_index_];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal.orientation.yaw);
    tf2::Transform transform(q, tf2::Vector3(goal.position.x, goal.position.y, goal.position.z));
    Eigen::Matrix3d rot_mat;
    rot_mat << transform.getBasis()[0][0], transform.getBasis()[0][1], transform.getBasis()[0][2],
               transform.getBasis()[1][0], transform.getBasis()[1][1], transform.getBasis()[1][2],
               transform.getBasis()[2][0], transform.getBasis()[2][1], transform.getBasis()[2][2];
    goal_in_global_mat_.block<3, 3>(0, 0) = rot_mat;
    goal_in_global_mat_.block<3, 1>(0, 3) = Eigen::Vector3d(goal.position.x, goal.position.y, goal.position.z);
    publishGoal();
    ROS_INFO("Patrol to point %lu: position=(%f, %f, %f), yaw=%f",
             current_patrol_index_ + 1, goal.position.x, goal.position.y, goal.position.z, goal.orientation.yaw);
}

Pose ROSInterface::getRobotInMapOrigin() {
    getRobotToMapOriginMat();
    Pose pose;
    pose.position.x = robot_in_map_origin_mat_(0, 3);
    pose.position.y = robot_in_map_origin_mat_(1, 3);
    pose.position.z = robot_in_map_origin_mat_(2, 3);
    Eigen::Matrix3d rot = robot_in_map_origin_mat_.block<3, 3>(0, 0);
    tf2::Matrix3x3 tf_rot(rot(0, 0), rot(0, 1), rot(0, 2),
                          rot(1, 0), rot(1, 1), rot(1, 2),
                          rot(2, 0), rot(2, 1), rot(2, 2));
    double roll, pitch, yaw;
    tf_rot.getRPY(roll, pitch, yaw);
    pose.orientation.yaw = yaw;
    return pose;
}