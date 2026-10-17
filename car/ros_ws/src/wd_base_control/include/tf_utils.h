#ifndef TF_UTILS_H
#define TF_UTILS_H

#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>
#include <string>

namespace tf_utils {

/// @brief 将 tf2::Transform 转换为 Eigen::Matrix4d
/// @param transform 输入的 tf2::Transform
/// @return 对应的 4x4 齐次变换矩阵
Eigen::Matrix4d transformToEigenMatrix(const tf2::Transform& transform);

/// @brief 将点从坐标系 A 转换到坐标系 B
/// @param tf_buffer tf2 变换缓冲区
/// @param source_frame 源坐标系
/// @param target_frame 目标坐标系
/// @param point 源坐标系中的点 (geometry_msgs::Point)
/// @param timeout 查找变换的超时时间
/// @return 转换到目标坐标系的点 (geometry_msgs::Point)
geometry_msgs::Point transformPoint(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& source_frame,
    const std::string& target_frame,
    const geometry_msgs::Point& point,
    const ros::Duration& timeout = ros::Duration(1.0));

/// @brief 将位姿从坐标系 A 转换到坐标系 B
/// @param tf_buffer tf2 变换缓冲区
/// @param source_frame 源坐标系
/// @param target_frame 目标坐标系
/// @param pose 源坐标系中的位姿 (geometry_msgs::Pose)
/// @param timeout 查找变换的超时时间
/// @return 转换到目标坐标系的位姿 (geometry_msgs::Pose)
geometry_msgs::Pose transformPose(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& source_frame,
    const std::string& target_frame,
    const geometry_msgs::Pose& pose,
    const ros::Duration& timeout = ros::Duration(1.0));

} // namespace tf_utils

#endif // TF_UTILS_H