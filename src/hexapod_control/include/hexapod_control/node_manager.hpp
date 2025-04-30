
// my_rviz_panel.hpp
#ifndef NODE_MANAGER_HPP
#define NODE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>

const char NODE_NAME[] = "hexapod_control_rviz_panel";

class NodeManager {
public:
  static rclcpp::Node::SharedPtr getNode();
};

#endif
