

#include "hexapod_msgs/msg/pose.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <ostream>

std::ostream &operator<<(std::ostream &os,
                         const hexapod_msgs::msg::Pose &pose) {
  os << pose.name << ":\n";
  for (size_t i = 0; i < pose.names.size(); i++) {
    os << " - " << pose.names[i] << "=\t[" << pose.positions[i].x << ","
       << pose.positions[i].y << "," << pose.positions[i].z << "]\n";
  }
  return os;
}

void RCLCPP_LOG_POSE(hexapod_msgs::msg::Pose pose) {
  RCLCPP_INFO(rclcpp::get_logger("TEST"), "%s :", pose.name.c_str());
  for (size_t i = 0; i < pose.names.size(); i++) {
    RCLCPP_INFO(rclcpp::get_logger("TEST"), " - %s = [%.4f,%.4f,%.4f]",
                pose.names[i].c_str(), pose.positions[i].x, pose.positions[i].y,
                pose.positions[i].z);
  }
}
