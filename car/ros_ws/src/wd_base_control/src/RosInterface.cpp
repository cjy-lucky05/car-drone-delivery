#include "RosInterface.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <actionlib_msgs/GoalID.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <nlohmann/json.hpp>
#include <std_msgs/UInt8MultiArray.h>

RosInterface::RosInterface(std::shared_ptr<RobotState> robot_state)
    : robot_state_(robot_state), tf_listener_(tf_buffer_), map_frame_("map"), robot_frame_("base_footprint"),
      current_patrol_index_(0), patrol_active_(false), K_rotation_(0.002), K_velocity_(0.0000045), max_velocity_(0.25) {
    ros::NodeHandle nh;
    map_sub_ = nh.subscribe("/map", 1, &RosInterface::mapCallback, this);
    object_markers_sub_ = nh.subscribe("/object_markers", 10, &RosInterface::markersCallback, this);
    drone_markers_sub_ = nh.subscribe("/drone_markers", 10, &RosInterface::droneMarkersCallback, this);
    patrol_sub_ = nh.subscribe("/patrol_cmd", 1, &RosInterface::patrolCallback, this);
    image_sub_ = nh.subscribe("/camera/image_raw", 1, &RosInterface::imageCallback, this);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    attack_pub_ = nh.advertise<std_msgs::Bool>("/attack_cmd", 1);
    cancel_pub_ = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    serial_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("/gimbal_cmd", 10); // 新增：云台控制发布

    nh.param("k_rotation", K_rotation_, 0.002);
    nh.param("k_velocity", K_velocity_, 0.0000045);
    nh.param("max_velocity", max_velocity_, 0.25);

    CLASSES_ = {"background", "aeroplane", "bicycle", "bird", "boat",
                "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                "sofa", "train", "tvmonitor"};
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    for (size_t i = 0; i < CLASSES_.size(); ++i) {
        COLORS_.push_back(cv::Scalar(dis(gen), dis(gen), dis(gen)));
    }
    std::string prototxt_path = "/home/bye/ros_car/src/wd_base_control/lib/trained_network/MobileNetSSD_deploy.prototxt.txt";
    std::string model_path = "/home/bye/ros_car/src/wd_base_control/lib/trained_network/MobileNetSSD_deploy.caffemodel";
    network_ = cv::dnn::readNetFromCaffe(prototxt_path, model_path);
    if (network_.empty()) {
        ROS_ERROR("Failed to load MobileNetSSD model.");
        throw std::runtime_error("Model loading failed");
    }
}

void RosInterface::publishGimbalCmd(int pwm_x, int pwm_y) {
    std::vector<uint8_t> data = {
        (uint8_t)((int)pwm_x >> 8 & 0xFF), (uint8_t)((int)pwm_x & 0xFF),
        (uint8_t)((int)pwm_y >> 8 & 0xFF), (uint8_t)((int)pwm_y & 0xFF)
    };
    std::vector<uint8_t> frame = build_frame(0x0D, data);

    std_msgs::UInt8MultiArray msg;
    msg.data = frame;
    serial_pub_.publish(msg);
    ROS_INFO("Published gimbal command: frame size=%lu", frame.size());
}

void RosInterface::fireOnce() {
    std_msgs::Bool msg;
    msg.data = true;
    attack_pub_.publish(msg);
    ROS_INFO("Published fire command");
}

void RosInterface::setPatrolPoints(const std::vector<Pose>& points) {
    patrol_points_ = points;
    current_patrol_index_ = 0;
    ROS_INFO("Updated patrol points: %lu points", points.size());
}

void RosInterface::patrolCallback(const std_msgs::Bool::ConstPtr& msg) {
    patrol_active_ = msg->data;
    if (patrol_active_ && !patrol_points_.empty()) {
        startPatrol(current_patrol_index_);
    } else if (!patrol_active_) {
        cancelGoal();
        ROS_INFO("Patrol stopped");
    }
}

void RosInterface::startPatrol(size_t index) {
    if (patrol_points_.empty()) {
        ROS_WARN("No patrol points set, cannot start patrol");
        patrol_active_ = false;
        return;
    }
    current_patrol_index_ = index % patrol_points_.size();
    patrol_active_ = true;
    publishGoal(patrol_points_[current_patrol_index_]);
    ROS_INFO("Patrol to point %lu: position=(%f, %f, %f), yaw=%f",
             current_patrol_index_ + 1, 
             patrol_points_[current_patrol_index_].position.x,
             patrol_points_[current_patrol_index_].position.y,
             patrol_points_[current_patrol_index_].position.z,
             patrol_points_[current_patrol_index_].orientation.yaw);
}

void RosInterface::publishGoal(const Pose& goal) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, goal.orientation.yaw);
    tf2::Transform transform(q, tf2::Vector3(goal.position.x, goal.position.y, goal.position.z));
    Eigen::Matrix3d rot_mat;
    rot_mat << transform.getBasis()[0][0], transform.getBasis()[0][1], transform.getBasis()[0][2],
               transform.getBasis()[1][0], transform.getBasis()[1][1], transform.getBasis()[1][2],
               transform.getBasis()[2][0], transform.getBasis()[2][1], transform.getBasis()[2][2];
    goal_in_global_mat_.block<3, 3>(0, 0) = rot_mat;
    goal_in_global_mat_.block<3, 1>(0, 3) = Eigen::Vector3d(goal.position.x, goal.position.y, goal.position.z);
    goal_pose_msg_.header.frame_id = map_frame_;
    goal_pose_msg_.header.stamp = ros::Time::now();
    goal_pose_msg_.pose.position.x = goal_in_global_mat_(0, 3);
    goal_pose_msg_.pose.position.y = goal_in_global_mat_(1, 3);
    goal_pose_msg_.pose.position.z = goal_in_global_mat_(2, 3);
    Eigen::Matrix3d rot = goal_in_global_mat_.block<3, 3>(0, 0);
    tf2::Matrix3x3 tf_rot(rot(0, 0), rot(0, 1), rot(0, 2),
                          rot(1, 0), rot(1, 1), rot(1, 2),
                          rot(2, 0), rot(2, 1), rot(2, 2));
    tf2::Quaternion q_out;
    tf_rot.getRotation(q_out);
    goal_pose_msg_.pose.orientation = tf2::toMsg(q_out);
    goal_pub_.publish(goal_pose_msg_);
}

void RosInterface::cancelGoal() {
    actionlib_msgs::GoalID cancel_msg;
    cancel_msg.stamp = ros::Time::now();
    cancel_pub_.publish(cancel_msg);
    geometry_msgs::PoseStamped empty_goal;
    empty_goal.header.frame_id = map_frame_;
    empty_goal.header.stamp = ros::Time::now();
    goal_pub_.publish(empty_goal);
    patrol_active_ = false;
    ROS_INFO("Navigation goal canceled");
}

bool RosInterface::isGoalReached() {
    if (current_patrol_index_ >= patrol_points_.size()) return false;
    Pose current_pose = getRobotInMapOrigin();
    const auto& goal = patrol_points_[current_patrol_index_];
    double dx = current_pose.position.x - goal.position.x;
    double dy = current_pose.position.y - goal.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.1;
}

void RosInterface::randomMove() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5, 0.5);
    geometry_msgs::Twist cmd;
    cmd.linear.x = dis(gen);
    cmd.angular.z = dis(gen);
    cmd_vel_pub_.publish(cmd);
    ROS_INFO("Random move: linear_x=%f, angular_z=%f", cmd.linear.x, cmd.angular.z);
}

void RosInterface::moveBaseResultCallback(const actionlib::SimpleClientGoalState& state,
                                         const move_base_msgs::MoveBaseResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        goal_reached_ = true;
        ROS_INFO("Goal reached successfully");
    } else {
        goal_reached_ = false;
        ROS_WARN("Goal failed: %s", state.toString().c_str());
    }
}

void RosInterface::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_msg_ = msg;
    getGlobalToMapOriginMat();
}

void RosInterface::markersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    object_markers_msg_ = msg;
    updateObjectPoseList();
}

void RosInterface::droneMarkersCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    drone_markers_msg_ = msg;
    updateObjectPoseList();
}

void RosInterface::updateObjectPoseList() {
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
}

void RosInterface::publishCmdVel(double linear, double angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_vel_pub_.publish(cmd);
    ROS_INFO("Published cmd_vel: linear_x=%f, angular_z=%f", linear, angular);
}

void RosInterface::getGlobalToMapOriginMat() {
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

void RosInterface::getRobotToMapOriginMat() {
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
    } catch (const tf2::LookupException& e) {
        ROS_WARN("Failed to get map to base_footprint transform: %s", e.what());
    }
}

Pose RosInterface::getRobotInMapOrigin() {
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

void RosInterface::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (robot_state_->getCurrentState() != State::TRACK && 
        robot_state_->getCurrentState() != State::PATROL && 
        robot_state_->getCurrentState() != State::GOTO_GOAL) return;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    int h = frame.rows;
    int w = frame.cols;

    cv::Mat frame_resized;
    cv::resize(frame, frame_resized, cv::Size(300, 300));
    cv::Mat blob = cv::dnn::blobFromImage(frame_resized, 0.007843, cv::Size(300, 300), 127.5);

    network_.setInput(blob);
    cv::Mat detections = network_.forward();

    bool person_detected = false;
    int area_biggest_detection = 0;
    int center_biggest_detection = 0;

    for (int i = 0; i < detections.size[2]; i++) {
        float* data = detections.ptr<float>(0, 0, i);
        int object_class = static_cast<int>(data[1]);
        float confidence = data[2];

        if (object_class == 15 && confidence > 0.2) {
            person_detected = true;
            int startX = static_cast<int>(data[3] * w);
            int startY = static_cast<int>(data[4] * h);
            int endX = static_cast<int>(data[5] * w);
            int endY = static_cast<int>(data[6] * h);

            int area_box = (endX - startX) * (endY - startY);
            int center_box = (startX + endX) / 2;

            cv::rectangle(frame, cv::Point(startX, startY), cv::Point(endX, endY), cv::Scalar(0, 0, 255), 2);
            std::string label = "person: " + std::to_string(int(confidence * 100)) + "%";
            cv::putText(frame, label, cv::Point(startX, startY - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

            if (area_box > area_biggest_detection) {
                area_biggest_detection = area_box;
                center_biggest_detection = center_box;
            }
        }
    }

    if (person_detected && area_biggest_detection > 10000) {
        robot_state_->setTargetDetected(true);
        robot_state_->setTargetArea(area_biggest_detection);
        if (robot_state_->getCurrentState() == State::PATROL) {
            DataQueMsg* msg = GetFreeBuffer(robot_state_->getCommandQueue(), 1000);
            if (msg) {
                nlohmann::json event_json = {
                    {"type", static_cast<int>(EventType::TARGET_DETECTED)},
                    {"data", ""}
                };
                std::string event_data = event_json.dump();
                size_t data_size = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
                std::memcpy(msg->buff, event_data.data(), data_size);
                PutReadyBuffer(robot_state_->getCommandQueue(), msg);
                ROS_INFO("Target detected, event triggered");
            }
        }
        if (robot_state_->getCurrentState() == State::TRACK) {
            double distance_to_person = 150000;
            double v = K_velocity_ * (distance_to_person - area_biggest_detection);
            double w = K_rotation_ * (w / 2 - center_biggest_detection);
            if (v > max_velocity_) v = max_velocity_;
            if (v < -max_velocity_) v = -max_velocity_;
            publishCmdVel(v, w);
        }
    } else if (robot_state_->getCurrentState() == State::TRACK) {
        robot_state_->setTargetDetected(false);
        robot_state_->setTargetArea(0.0);
        DataQueMsg* msg = GetFreeBuffer(robot_state_->getCommandQueue(), 1000);
        if (msg) {
            nlohmann::json event_json = {
                {"type", static_cast<int>(EventType::TARGET_LOST)},
                {"data", ""}
            };
            std::string event_data = event_json.dump();
            size_t data_size = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), data_size);
            PutReadyBuffer(robot_state_->getCommandQueue(), msg);
            ROS_INFO("Target lost, event triggered");
        }
    }

    cv::imshow("Tracking", frame);
    cv::waitKey(1);
}