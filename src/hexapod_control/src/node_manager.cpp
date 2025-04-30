#include "hexapod_control/node_manager.hpp"
#include <rclcpp/node.hpp>

rclcpp::Node::SharedPtr NodeManager::getNode() {
  static rclcpp::Node::SharedPtr instance =
      std::make_shared<rclcpp::Node>(NODE_NAME);
  return instance;
}
