#ifndef HEXAPOD_COMMON_LOGGING_HPP
#define HEXAPOD_COMMON_LOGGING_HPP

#include "hexapod_msgs/msg/pose.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

void RCLCPP_LOG_POSE(hexapod_msgs::msg::Pose pose);

std::ostream &operator<<(std::ostream &os, const hexapod_msgs::msg::Pose &pose);

#endif
