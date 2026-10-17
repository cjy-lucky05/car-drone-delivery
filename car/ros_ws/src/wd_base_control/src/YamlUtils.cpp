#include "YamlUtils.h"
#include <fstream>
#include <iostream>

YamlUtils::YamlUtils(const std::string& file_path) : file_path_(file_path) {}

YAML::Node YamlUtils::loadYamlFile() {
    try {
        return YAML::LoadFile(file_path_);
    } catch (const YAML::Exception& e) {
        std::cerr << "加载YAML文件失败: " << e.what() << std::endl;
        return YAML::Node();
    }
}

void YamlUtils::saveYamlFile(const YAML::Node& node) {
    try {
        std::ofstream fout(file_path_);
        if (fout.is_open()) {
            fout << node;
            fout.close();
        } else {
            std::cerr << "保存YAML文件失败: " << file_path_ << std::endl;
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "保存YAML文件失败: " << e.what() << std::endl;
    }
}

RobotConfig YamlUtils::loadConfig() {
    RobotConfig config;
    YAML::Node node = loadYamlFile();

    // 巡逻点
    if (node["patrol_points"] && node["patrol_points"].IsSequence()) {
        for (const auto& p : node["patrol_points"]) {
            Pose point;
            point.position.x = p["x"] ? p["x"].as<double>() : 0.0;
            point.position.y = p["y"] ? p["y"].as<double>() : 0.0;
            point.position.z = p["z"] ? p["z"].as<double>() : 0.0;
            point.orientation.yaw = p["yaw"] ? p["yaw"].as<double>() : 0.0;
            config.patrol_points.push_back(point);
        }
    }

    // 目标点
    if (node["goal_point"] && node["goal_point"].IsMap()) {
        config.goal_point.position.x = node["goal_point"]["x"] ? node["goal_point"]["x"].as<double>() : 0.0;
        config.goal_point.position.y = node["goal_point"]["y"] ? node["goal_point"]["y"].as<double>() : 0.0;
        config.goal_point.position.z = node["goal_point"]["z"] ? node["goal_point"]["z"].as<double>() : 0.0;
        config.goal_point.orientation.yaw = node["goal_point"]["yaw"] ? node["goal_point"]["yaw"].as<double>() : 0.0;
    }

    // 自动巡逻开关
    config.enable_auto_patrol = node["enable_auto_patrol"] ? node["enable_auto_patrol"].as<bool>() : false;

    // 巡逻模式（只读，设置默认值）
    if (node["patrol_mode"] && node["patrol_mode"].IsMap()) {
        config.patrol_mode.mode = node["patrol_mode"]["mode"] ? node["patrol_mode"]["mode"].as<std::string>() : "auto";
        config.patrol_mode.alert = node["patrol_mode"]["alert"] ? node["patrol_mode"]["alert"].as<bool>() : false;
        config.patrol_mode.tracking_timeout = node["patrol_mode"]["tracking_timeout"] ? node["patrol_mode"]["tracking_timeout"].as<int>() : 10;
    } else {
        config.patrol_mode = {"auto", false, 10};
    }

    // TCP配置（只读，设置默认值）
    if (node["tcp_config"] && node["tcp_config"].IsMap()) {
        config.tcp_config.ip = node["tcp_config"]["ip"] ? node["tcp_config"]["ip"].as<std::string>() : "0.0.0.0";
        config.tcp_config.port = node["tcp_config"]["port"] ? node["tcp_config"]["port"].as<int>() : 9000;
    } else {
        config.tcp_config = {"0.0.0.0", 9000};
    }

    // HTTP配置（只读，设置默认值）
    if (node["http_config"] && node["http_config"].IsMap()) {
        config.http_config.enable = node["http_config"]["enable"] ? node["http_config"]["enable"].as<bool>() : false;
        config.http_config.ip = node["http_config"]["ip"] ? node["http_config"]["ip"].as<std::string>() : "0.0.0.0";
        config.http_config.port = node["http_config"]["port"] ? node["http_config"]["port"].as<int>() : 8080;
    } else {
        config.http_config = {false, "0.0.0.0", 8080};
    }

    // 视频配置（只读，设置默认值）
    if (node["video_config"] && node["video_config"].IsMap()) {
        config.video_config.ip = node["video_config"]["ip"] ? node["video_config"]["ip"].as<std::string>() : "0.0.0.0";
        config.video_config.port = node["video_config"]["port"] ? node["video_config"]["port"].as<int>() : 9000;
    } else {
        config.video_config = {"0.0.0.0", 9000};
    }

    // 日志配置（只读，设置默认值）
    if (node["log_config"] && node["log_config"].IsMap()) {
        config.log_config.enable = node["log_config"]["enable"] ? node["log_config"]["enable"].as<bool>() : false;
        config.log_config.file = node["log_config"]["file"] ? node["log_config"]["file"].as<std::string>() : "/tmp/defense.log";
        config.log_config.max_size = node["log_config"]["max_size"] ? node["log_config"]["max_size"].as<std::string>() : "10MB";
        config.log_config.rotate = node["log_config"]["rotate"] ? node["log_config"]["rotate"].as<int>() : 5;
    } else {
        config.log_config = {false, "/tmp/defense.log", "10MB", 5};
    }

    return config;
}

void YamlUtils::saveConfig(const RobotConfig& config) {
    YAML::Node node = loadYamlFile(); // 加载原始文件以保留只读部分

    // 更新巡逻点（可写）
    YAML::Node points;
    for (const auto& p : config.patrol_points) {
        YAML::Node point;
        point["x"] = p.position.x;
        point["y"] = p.position.y;
        point["z"] = p.position.z;
        point["yaw"] = p.orientation.yaw;
        points.push_back(point);
    }
    node["patrol_points"] = points;

    // 更新目标点（可写）
    YAML::Node goal;
    goal["x"] = config.goal_point.position.x;
    goal["y"] = config.goal_point.position.y;
    goal["z"] = config.goal_point.position.z;
    goal["yaw"] = config.goal_point.orientation.yaw;
    node["goal_point"] = goal;

    // 更新自动巡逻开关（可写）
    node["enable_auto_patrol"] = config.enable_auto_patrol;

    // 保留只读部分
    if (!node["patrol_mode"]) node["patrol_mode"] = YAML::Node();
    if (!node["tcp_config"]) node["tcp_config"] = YAML::Node();
    if (!node["http_config"]) node["http_config"] = YAML::Node();
    if (!node["video_config"]) node["video_config"] = YAML::Node();
    if (!node["log_config"]) node["log_config"] = YAML::Node();

    saveYamlFile(node);
}

std::vector<Pose> YamlUtils::loadPatrolPoints() {
    std::vector<Pose> patrol_points;
    YAML::Node node = loadYamlFile();

    if (node["patrol_points"] && node["patrol_points"].IsSequence()) {
        for (const auto& p : node["patrol_points"]) {
            Pose point;
            point.position.x = p["x"] ? p["x"].as<double>() : 0.0;
            point.position.y = p["y"] ? p["y"].as<double>() : 0.0;
            point.position.z = p["z"] ? p["z"].as<double>() : 0.0;
            point.orientation.yaw = p["yaw"] ? p["yaw"].as<double>() : 0.0;
            patrol_points.push_back(point);
        }
    }

    return patrol_points;
}

void YamlUtils::savePatrolPoints(const std::vector<Pose>& patrol_points) {
    YAML::Node node = loadYamlFile(); // 加载原始文件以保留其他部分

    // 更新巡逻点
    YAML::Node points;
    for (const auto& p : patrol_points) {
        YAML::Node point;
        point["x"] = p.position.x;
        point["y"] = p.position.y;
        point["z"] = p.position.z;
        point["yaw"] = p.orientation.yaw;
        points.push_back(point);
    }
    node["patrol_points"] = points;

    saveYamlFile(node);
}

void YamlUtils::savePatrolState(const Pose& goal_point) {
    YAML::Node node = loadYamlFile(); // 加载原始文件以保留其他部分

    // 更新目标点
    YAML::Node goal;
    goal["x"] = goal_point.position.x;
    goal["y"] = goal_point.position.y;
    goal["z"] = goal_point.position.z;
    goal["yaw"] = goal_point.orientation.yaw;
    node["goal_point"] = goal;

    saveYamlFile(node);
}

bool YamlUtils::loadAutoPatrol() {
    YAML::Node node = loadYamlFile();
    return node["enable_auto_patrol"] ? node["enable_auto_patrol"].as<bool>() : false;
}

void YamlUtils::saveAutoPatrol(bool enable_auto_patrol) {
    YAML::Node node = loadYamlFile(); // 加载原始文件以保留其他部分
    node["enable_auto_patrol"] = enable_auto_patrol;
    saveYamlFile(node);
}