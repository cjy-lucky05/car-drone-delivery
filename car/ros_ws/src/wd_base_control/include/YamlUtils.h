#ifndef YAML_UTILS_H
#define YAML_UTILS_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <map>
#include "types.h"

struct PatrolMode {
    std::string mode;
    bool alert;
    int tracking_timeout;
};

struct TcpConfig {
    std::string ip;
    int port;
};

struct HttpConfig {
    bool enable;
    std::string ip;
    int port;
};

struct VideoConfig {
    std::string ip;
    int port;
};

struct LogConfig {
    bool enable;
    std::string file;
    std::string max_size;
    int rotate;
};

struct RobotConfig {
    std::vector<Pose> patrol_points; // 使用 Pose 表示巡逻点
    Pose goal_point; // 使用 Pose 表示目标点
    bool enable_auto_patrol;
    PatrolMode patrol_mode; // 原 AttackMode
    TcpConfig tcp_config; // 原 TcpServer
    HttpConfig http_config; // 原 HttpServer
    VideoConfig video_config; // 原 VideoServer
    LogConfig log_config; // 原 Log
};

class YamlUtils {
public:
    // 构造函数，指定配置文件路径
    explicit YamlUtils(const std::string& file_path);

    // 加载整个配置文件
    RobotConfig loadConfig();

    // 保存整个配置
    void saveConfig(const RobotConfig& config);

    // 单独加载巡逻点
    std::vector<Pose> loadPatrolPoints();

    // 单独保存巡逻点
    void savePatrolPoints(const std::vector<Pose>& patrol_points);

    // 单独保存巡逻状态（目标点）
    void savePatrolState(const Pose& goal_point);

    // 单独加载自动巡逻开关
    bool loadAutoPatrol();

    // 单独保存自动巡逻开关
    void saveAutoPatrol(bool enable_auto_patrol);

private:
    std::string file_path_; // 配置文件路径
    YAML::Node loadYamlFile(); // 内部加载YAML文件
    void saveYamlFile(const YAML::Node& node); // 内部保存YAML文件
};

#endif // YAML_UTILS_H