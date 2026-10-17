#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <random>
#include <iostream>

class HumanTracker {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;

    cv::dnn::Net network_;
    std::vector<std::string> CLASSES_;
    std::vector<cv::Scalar> COLORS_;
    cv::VideoCapture cap_;   // 摄像头对象

    double K_rotation_ = 0.002;
    double K_velocity_ = 0.0000045;
    double max_velocity_ = 0.25;

public:
    HumanTracker() {
        // 发布 /cmd_vel
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // 打开摄像头 0
        cap_.open(1);
        if (!cap_.isOpened()) {
            throw std::runtime_error("无法打开摄像头 0");
        }
        // 设置分辨率 640x480
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

        ROS_INFO("HumanTracker 节点初始化完成，摄像头已打开");
    }

    void processFrame() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) return;

        int h = frame.rows;
        int w = frame.cols;

        // 预处理
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(300, 300));
        cv::Mat blob = cv::dnn::blobFromImage(frame_resized, 0.007843, cv::Size(300, 300), 127.5);

        // 前向传播
        network_.setInput(blob);
        cv::Mat detections = network_.forward();

        int person_detected = 0;
        int area_biggest_detection = 0;
        int center_biggest_detection = 0;

        for (int i = 0; i < detections.size[2]; i++) {
            float* data = detections.ptr<float>(0, 0, i);
            int object_class = static_cast<int>(data[1]);
            float confidence = data[2];

            if (object_class == 15 && confidence > 0.2) { // person
                person_detected = 1;
                int startX = static_cast<int>(data[3] * w);
                int startY = static_cast<int>(data[4] * h);
                int endX   = static_cast<int>(data[5] * w);
                int endY   = static_cast<int>(data[6] * h);

                int area_box = (endX - startX) * (endY - startY);
                int center_box = (startX + endX) / 2;

                cv::rectangle(frame, cv::Point(startX, startY), cv::Point(endX, endY),
                              cv::Scalar(0, 0, 255), 2);
                std::string label = "person: " + std::to_string(int(confidence * 100)) + "%";
                cv::putText(frame, label, cv::Point(startX, startY - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);

                if (area_box > area_biggest_detection) {
                    area_biggest_detection = area_box;
                    center_biggest_detection = center_box;
                }
            }
        }

        if (person_detected && area_biggest_detection > 10000) {
            double distance_to_person = 150000;
            double v = K_velocity_ * (distance_to_person - area_biggest_detection);
            double w_cmd = K_rotation_ * (w / 2 - center_biggest_detection);

            if (v > max_velocity_) v = max_velocity_;
            if (v < -max_velocity_) v = -max_velocity_;

            sendCommand(v, w_cmd);
        }

        cv::imshow("Tracking", frame);
        cv::waitKey(1);
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
        ros::Rate loop(30);   // 30 FPS
        while (ros::ok()) {
            tracker.processFrame();
            ros::spinOnce();
            loop.sleep();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}
