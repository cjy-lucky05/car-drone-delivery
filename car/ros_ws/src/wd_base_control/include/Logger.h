#ifndef LOGGER_H
#define LOGGER_H

#include "types.h"
#include "RobotState.h"
#include <string>
#include <memory>

class Logger {
public:
    Logger(const std::string& log_file);

    void logEvent(const std::string& event, const Pose& pose);
    std::string exportLogs(const std::string& start_time, const std::string& end_time);

private:
    std::string log_file_;
};

#endif // LOGGER_H