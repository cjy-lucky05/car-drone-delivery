#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <vector>
#include <random>
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>

class HumanTracker {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    cv::dnn::Net network_;
    std::vector<std::string> CLASSES_;
    std::vector<cv::Scalar> COLORS_;
    cv::VideoCapture cap_;
    cv::Ptr<cv::TrackerCSRT> tracker_;
    std::mutex frame_mutex_;
    std::atomic<bool> tracker_initialized_{false};
    std::atomic<bool> running_{true};

    double K_rotation_ = 0.002;
    double K_velocity_ = 0.0000045;
    double max_velocity_ = 0.25;
    const int DETECTION_INTERVAL = 10; // 每10帧重新检测
    int frame_count_ = 0;

public:
    HumanTracker() {
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // 打开摄像头
        cap_.open(1);
        if (!cap_.isOpened()) {
            throw std::runtime_error("无法打开摄像头 1");
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        // 类别标签
        CLASSES_ = {"background", "aeroplane", "bicycle", "bird", "boat",
                    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                    "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                    "sofa", "train", "tvmonitor"};

        // 随机颜色
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 255);
        for (size_t i = 0; i < CLASSES_.size(); ++i) {
            COLORS_.push_back(cv::Scalar(dis(gen), dis(gen), dis(gen)));
        }

        // 加载 MobileNetSSD 模型
        std::string prototxt_path = "/home/bingda/catkin_ws/src/wd_base_control/lib/trained_network/MobileNetSSD_deploy.prototxt.txt";
        std::string model_path = "/home/bingda/catkin_ws/src/wd_base_control/lib/trained_network/MobileNetSSD_deploy.caffemodel";
        network_ = cv::dnn::readNetFromCaffe(prototxt_path, model_path);
        if (network_.empty()) {
            throw std::runtime_error("加载 MobileNetSSD 模型失败");
        }

        // 初始化 CSRT 跟踪器
        tracker_ = cv::TrackerCSRT::create();

        ROS_INFO("HumanTracker 节点初始化完成，摄像头已打开");
    }

    ~HumanTracker() {
        running_ = false;
        cap_.release();
        cv::destroyAllWindows();
    }

    void captureThread() {
        cv::Mat frame;
        while (running_ && ros::ok()) {
            cap_ >> frame;
            if (!frame.empty()) {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                frame.copyTo(current_frame_);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void processFrame() {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (current_frame_.empty()) return;
            current_frame_.copyTo(frame);
        }

        int h = frame.rows;
        int w = frame.cols;
        cv::Rect2d bbox;

        if (!tracker_initialized_ || frame_count_ % DETECTION_INTERVAL == 0) {
            // 进行目标检测
            cv::Mat frame_resized;
            cv::resize(frame, frame_resized, cv::Size(300, 300));
            cv::Mat blob = cv::dnn::blobFromImage(frame_resized, 0.007843, cv::Size(300, 300), 127.5);
            network_.setInput(blob);
            cv::Mat detections = network_.forward();

            int max_area = 0;
            cv::Rect best_box;
            float best_confidence = 0;

            for (int i = 0; i < detections.size[2]; i++) {
                float* data = detections.ptr<float>(0, 0, i);
                int object_class = static_cast<int>(data[1]);
                float confidence = data[2];

                if (object_class == 15 && confidence > 0.2) { // person
                    int startX = static_cast<int>(data[3] * w);
                    int startY = static_cast<int>(data[4] * h);
                    int endX = static_cast<int>(data[5] * w);
                    int endY = static_cast<int>(data[6] * h);
                    int area = (endX - startX) * (endY - startY);

                    if (area > max_area) {
                        max_area = area;
                        best_box = cv::Rect(startX, startY, endX - startX, endY - startY);
                        best_confidence = confidence;
                    }
                }
            }

            if (max_area > 10000) {
                bbox = best_box;
                if (!tracker_initialized_) {
                    tracker_->init(frame, bbox);
                    tracker_initialized_ = true;
                } else {
                    tracker_->init(frame, bbox); // 重新初始化跟踪器
                }

                cv::rectangle(frame, best_box, cv::Scalar(0, 0, 255), 2);
                std::string label = "person: " + std::to_string(int(best_confidence * 100)) + "%";
                cv::putText(frame, label, cv::Point(best_box.x, best_box.y - 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            }
        } else if (tracker_initialized_) {
            // 使用 CSRT 跟踪
            bool ok = tracker_->update(frame, bbox);
            if (ok) {
                cv::rectangle(frame, bbox, cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, "Tracking", cv::Point(bbox.x, bbox.y - 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            } else {
                tracker_initialized_ = false; // 跟踪失败，重新检测
            }
        }

        if (tracker_initialized_ && bbox.area() > 10000) {
            double distance_to_person = 150000;
            double v = K_velocity_ * (distance_to_person - bbox.area());
            double w_cmd = K_rotation_ * (w / 2 - (bbox.x + bbox.width / 2));

            if (v > max_velocity_) v = max_velocity_;
            if (v < -max_velocity_) v = -max_velocity_;

            sendCommand(v, w_cmd);
        }

        cv::imshow("Tracking", frame);
        cv::waitKey(1);
        frame_count_++;
    }

    void sendCommand(double v, double w) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = v;
        cmd.angular.z = w;
        cmd_pub_.publish(cmd);
    }

private:
    cv::Mat current_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "human_tracker");

    try {
        HumanTracker tracker;
        std::thread capture_thread(&HumanTracker::captureThread, &tracker);
        ros::Rate loop(30);

        while (ros::ok()) {
            tracker.processFrame();
            ros::spinOnce();
            loop.sleep();
        }

        capture_thread.join();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}