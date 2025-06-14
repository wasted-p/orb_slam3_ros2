#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <hexapod_common/hexapod.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_msgs/srv/set_joint_state.hpp>
#include <map>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <stdexcept>
#include <string>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

void parseYaml(std::string &yaml_string, std::map<std::string, double> &map) {
  try {
    YAML::Node root = YAML::Load(yaml_string);
    for (const auto &pair : root["initial_positions"]) {
      std::string joint_name = pair.first.as<std::string>();
      double value = pair.second.as<double>();
      map[joint_name] = value;
    }
  } catch (const YAML::Exception &e) {
    throw std::runtime_error(e.what());
  }
}

class JointStatePublisher : public rclcpp::Node {
public:
  JointStatePublisher() : Node("joint_state_publisher") { setupROS(); }
  ~JointStatePublisher() {};

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Service<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_state_service_;
  sensor_msgs::msg::JointState joint_state_msg_;
  std::map<std::string, double> initial_positions_;

  void timerCallback() {
    joint_state_msg_.header.stamp = get_clock()->now();
    joint_state_publisher_->publish(joint_state_msg_);
  }

  void setupROS() {
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC,
                                                             rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JointStatePublisher::timerCallback, this));
    joint_state_msg_.header.frame_id = "base_footprint";

    set_joint_state_service_ = create_service<hexapod_msgs::srv::SetJointState>(
        SET_JOINT_STATE_SERVICE_NAME,
        std::bind(&JointStatePublisher::handleSetJointStateRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Declare and get the YAML string parameter
    std::string yaml_string = this->declare_parameter(
        "initial_positions", std::string("{initial_positions:}"));

    parseYaml(yaml_string, initial_positions_);
    RCLCPP_INFO(this->get_logger(), "Loaded %zu initial positions",
                initial_positions_.size());

    RCLCPP_INFO(get_logger(), "Initialising joints:");
    for (const auto &entry : initial_positions_) {
      RCLCPP_INFO(get_logger(), " - %s=%.2f", entry.first.c_str(),
                  entry.second);
      joint_state_msg_.name.push_back(entry.first);
      joint_state_msg_.position.push_back(entry.second);
    }
  }

  void handleSetJointStateRequest(
      const std::shared_ptr<hexapod_msgs::srv::SetJointState::Request> request,
      std::shared_ptr<hexapod_msgs::srv::SetJointState::Response> response) {
    RCLCPP_DEBUG(get_logger(), "SetJointState Request Recieved");

    joint_state_msg_ = request->joint_state;
    joint_state_msg_.header.stamp = this->now();
    response->success = true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
