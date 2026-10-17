#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <vector>
#include <random>
#include <iostream>
#include <cmath>  // 新增：用于角度计算
#include "config.h"

class HumanTracker {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber image_sub_;

    cv::dnn::Net network_;
    std::vector<std::string> CLASSES_;
    std::vector<cv::Scalar> COLORS_;

#if USE_REAL_CAMERA
    cv::VideoCapture cap_;
#endif

    std::vector<float> lidar_ranges_;
    float lidar_angle_min_ = 0.0f;
    float lidar_angle_increment_ = 0.0f;
    cv::Ptr<cv::Tracker> tracker_;
    cv::Rect2d current_bbox_;
    int frame_counter_ = 0;
    double v_filtered_prev_ = 0.0;
    double w_filtered_prev_ = 0.0;

public:
    HumanTracker() {
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        lidar_sub_ = nh_.subscribe("/scan", 1, &HumanTracker::lidarCallback, this);

#if USE_REAL_CAMERA
        cap_.open(1);
        if (!cap_.isOpened()) {
            throw std::runtime_error("无法打开摄像头 1");
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
#else
        image_sub_ = nh_.subscribe("/camera/image_raw", 1, &HumanTracker::imageCallback, this);
#endif

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

        std::string prototxt_path = PROTOTXT_PATH;
        std::string model_path = MODEL_PATH;

        network_ = cv::dnn::readNetFromCaffe(prototxt_path, model_path);
        if (network_.empty()) {
            throw std::runtime_error("加载 MobileNetSSD 模型失败");
        }

        tracker_ = cv::TrackerCSRT::create();
        ROS_INFO("HumanTracker 节点初始化完成");
    }

    void lidarCallback(const sensor_msgs::LaserScanConstPtr& msg) {
        lidar_ranges_ = msg->ranges;
        lidar_angle_min_ = msg->angle_min;
        lidar_angle_increment_ = msg->angle_increment;
    }

#if !USE_REAL_CAMERA
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            processFrame(frame);
            cv::imshow("Tracking", frame);
            cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
#endif

#if USE_REAL_CAMERA
    void processFrame() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        processFrame(frame);
        cv::imshow("Tracking", frame);
        cv::waitKey(1);
    }
#endif

    void processFrame(cv::Mat& frame) {
        int h = frame.rows;
        int w = frame.cols;

        frame_counter_++;
        bool target_tracked = false;
        int area_biggest_detection = 0;
        int center_biggest_detection = 0;

        bool run_detection = (frame_counter_ % DETECT_INTERVAL == 0 || tracker_.empty());

        if (run_detection) {
            cv::Mat frame_resized;
            cv::resize(frame, frame_resized, cv::Size(300, 300));
            cv::Mat blob = cv::dnn::blobFromImage(frame_resized, 0.007843, cv::Size(300, 300), 127.5);

            network_.setInput(blob);
            cv::Mat detections = network_.forward();

            cv::Rect2d best_bbox;
            float best_confidence = 0.0f;
            int person_detected = 0;

            for (int i = 0; i < detections.size[2]; i++) {
                float* data = detections.ptr<float>(0, 0, i);
                int object_class = static_cast<int>(data[1]);
                float confidence = data[2];

                if (object_class == 15 && confidence > 0.1) {
                    person_detected = 1;
                    double startX = data[3] * w;
                    double startY = data[4] * h;
                    double endX = data[5] * w;
                    double endY = data[6] * h;

                    cv::Rect2d bbox(startX, startY, endX - startX, endY - startY);
                    int area_box = static_cast<int>(bbox.area());

                    if (area_box > area_biggest_detection) {
                        area_biggest_detection = area_box;
                        center_biggest_detection = static_cast<int>((startX + endX) / 2);
                        best_bbox = bbox;
                        best_confidence = confidence;
                    }
                }
            }

            if (person_detected && area_biggest_detection > MIN_AREA_THRESHOLD) {
                tracker_ = cv::TrackerCSRT::create();
                tracker_->init(frame, best_bbox);
                current_bbox_ = best_bbox;
                target_tracked = true;

                cv::rectangle(frame, best_bbox, cv::Scalar(0, 0, 255), 2);
                std::string label = "person: " + std::to_string(int(best_confidence * 100)) + "%";
                cv::putText(frame, label, cv::Point(static_cast<int>(best_bbox.x), static_cast<int>(best_bbox.y - 5)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

                ROS_INFO("Detection: New target initialized, area=%d, confidence=%.2f, bbox=[%.1f, %.1f, %.1f, %.1f]",
                         area_biggest_detection, best_confidence, best_bbox.x, best_bbox.y,
                         best_bbox.x + best_bbox.width, best_bbox.y + best_bbox.height);
            } else if (person_detected) {
                ROS_WARN("Detection: Person detected but area too small (%d < %d)", area_biggest_detection, MIN_AREA_THRESHOLD);
            } else {
                ROS_WARN("Detection: No person detected");
            }
        } else {
            if (!tracker_.empty()) {
                if (tracker_->update(frame, current_bbox_)) {
                    area_biggest_detection = static_cast<int>(current_bbox_.area());
                    center_biggest_detection = static_cast<int>(current_bbox_.x + current_bbox_.width / 2);
                    if (area_biggest_detection > MIN_AREA_THRESHOLD) {
                        target_tracked = true;

                        cv::rectangle(frame, current_bbox_, cv::Scalar(0, 255, 0), 2);
                        cv::putText(frame, "tracking", cv::Point(static_cast<int>(current_bbox_.x), static_cast<int>(current_bbox_.y - 5)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

                        ROS_INFO("Tracking: Target updated, area=%d, center_x=%d", area_biggest_detection, center_biggest_detection);
                    } else {
                        ROS_WARN("Tracking: Area too small (%d < %d)", area_biggest_detection, MIN_AREA_THRESHOLD);
                        tracker_.release();
                    }
                } else {
                    ROS_WARN("Tracking failed, resetting tracker");
                    tracker_.release();
                }
            }
        }

        double v_filtered = 0.0;
        double w_filtered = 0.0;

        if (target_tracked) {
            double distance_to_person = ASSUMED_DISTANCE_TO_PERSON;
            double v_new = K_VELOCITY * (distance_to_person - area_biggest_detection);
            double w_new = K_ROTATION * (w / 2 - center_biggest_detection);

            if (v_new > MAX_VELOCITY) v_new = MAX_VELOCITY;
            if (v_new < -MAX_VELOCITY) v_new = -MAX_VELOCITY;

            v_filtered = ALPHA * v_new + (1 - ALPHA) * v_filtered_prev_;
            w_filtered = ALPHA * w_new + (1 - ALPHA) * w_filtered_prev_;

            v_filtered_prev_ = v_filtered;
            w_filtered_prev_ = w_filtered;

            ROS_INFO("Tracking active: v_filtered=%.2f, w_filtered=%.2f", v_filtered, w_filtered);

            // 新增：仅在有目标时应用避障
            adjustForObstacle(v_filtered, w_filtered, v_new >= 0 ? "forward" : "backward");
        } else {
            v_filtered = v_filtered_prev_ * DECELERATION_FACTOR;
            w_filtered = 0.0;

            v_filtered_prev_ = v_filtered;
            w_filtered_prev_ = w_filtered;

            ROS_INFO("No valid target: decelerating, v_filtered=%.2f", v_filtered);
        }

        sendCommand(v_filtered, w_filtered);
    }

    void adjustForObstacle(double& v, double& w, const std::string& direction) {
        if (lidar_ranges_.empty()) {
            ROS_WARN("No LiDAR data available");
            return;
        }

        int num_ranges = lidar_ranges_.size();
        if (lidar_angle_increment_ == 0.0f) {
            ROS_WARN("Invalid LiDAR angle increment");
            return;
        }

        // 转换为弧度
        const double half_angle = OBSTACLE_SECTOR_ANGLE * M_PI / 180.0;
        double center_angle = (direction == "forward") ? 0.0 : M_PI;  // 前方 0°，后方 180°

        // 计算检查的索引范围
        int min_index = static_cast<int>((center_angle - half_angle - lidar_angle_min_) / lidar_angle_increment_);
        int max_index = static_cast<int>((center_angle + half_angle - lidar_angle_min_) / lidar_angle_increment_);

        // 防止越界
        min_index = std::max(0, min_index);
        max_index = std::min(num_ranges - 1, max_index);

        // 找到最小距离
        float min_front_dist = 10.0f;
        for (int i = min_index; i <= max_index; ++i) {
            if (lidar_ranges_[i] < min_front_dist && lidar_ranges_[i] > 0.0f) {
                min_front_dist = lidar_ranges_[i];
            }
        }

        if (min_front_dist < OBSTACLE_DISTANCE_THRESHOLD) {
            v = 0.0;  // 停止前进或后退，保持角速度
            ROS_INFO("Obstacle detected in %s direction at %.2f m, stopping motion", direction.c_str(), min_front_dist);
        }
    }

    void sendCommand(double v, double w) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;
        cmd_pub_.publish(cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "human_tracker");

    try {
        HumanTracker tracker;
        ros::Rate loop(30);

#if USE_REAL_CAMERA
        while (ros::ok()) {
            tracker.processFrame();
            ros::spinOnce();
            loop.sleep();
        }
#else
        ros::spin();
#endif
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}