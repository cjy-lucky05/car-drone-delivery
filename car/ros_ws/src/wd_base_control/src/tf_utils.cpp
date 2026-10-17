#include "tf_utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf_utils {

Eigen::Matrix4d transformToEigenMatrix(const tf2::Transform& transform) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_mat;
    rot_mat << transform.getBasis()[0][0], transform.getBasis()[0][1], transform.getBasis()[0][2],
               transform.getBasis()[1][0], transform.getBasis()[1][1], transform.getBasis()[1][2],
               transform.getBasis()[2][0], transform.getBasis()[2][1], transform.getBasis()[2][2];
    mat.block<3, 3>(0, 0) = rot_mat;
    mat.block<3, 1>(0, 3) = Eigen::Vector3d(
        transform.getOrigin().x(),
        transform.getOrigin().y(),
        transform.getOrigin().z());
    return mat;
}

geometry_msgs::Point transformPoint(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& source_frame,
    const std::string& target_frame,
    const geometry_msgs::Point& point,
    const ros::Duration& timeout) {
    geometry_msgs::PointStamped point_stamped;
    point_stamped.header.frame_id = source_frame;
    point_stamped.header.stamp = ros::Time(0);
    point_stamped.point = point;

    geometry_msgs::PointStamped transformed_point;
    tf_buffer.transform(point_stamped, transformed_point, target_frame, timeout);
    return transformed_point.point;
}

geometry_msgs::Pose transformPose(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& source_frame,
    const std::string& target_frame,
    const geometry_msgs::Pose& pose,
    const ros::Duration& timeout) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = source_frame;
    pose_stamped.header.stamp = ros::Time(0);
    pose_stamped.pose = pose;

    geometry_msgs::PoseStamped transformed_pose;
    tf_buffer.transform(pose_stamped, transformed_pose, target_frame, timeout);
    return transformed_pose.pose;
}

} // namespace tf_utils