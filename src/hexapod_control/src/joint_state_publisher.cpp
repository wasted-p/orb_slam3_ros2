#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <hexapod_common/hexapod.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_common/yaml_utils.hpp>
#include <hexapod_msgs/srv/set_joint_state.hpp>
#include <map>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <string>

using namespace std::chrono_literals;

class JointStatePublisher : public rclcpp::Node {
public:
  JointStatePublisher() : Node("joint_state_publisher") { setupROS(); }
  ~JointStatePublisher() {};

private:
  std::string prefix_;
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
    std::string topic_name, service_name, yaml_string;

    // Declare and get the YAML string parameter
    yaml_string = this->declare_parameter("initial_positions",
                                          std::string("{initial_positions:}"));

    parseYamls(yaml_string, initial_positions_);

    // Declare and get the YAML string parameter
    prefix_ = this->declare_parameter("prefix", std::string(""));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&JointStatePublisher::timerCallback, this));

    topic_name = "/" + prefix_ + "/" + JOINT_STATE_TOPIC;
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(topic_name,
                                                             rclcpp::QoS(10));

    service_name = "/" + prefix_ + "/" + SET_JOINT_STATE_SERVICE_NAME;
    set_joint_state_service_ = create_service<hexapod_msgs::srv::SetJointState>(
        service_name,
        std::bind(&JointStatePublisher::handleSetJointStateRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    for (const auto &entry : initial_positions_) {
      joint_state_msg_.name.push_back(entry.first);
      joint_state_msg_.position.push_back(entry.second);
    }

    joint_state_msg_.header.frame_id = "base_footprint";
  }

  void handleSetJointStateRequest(
      const std::shared_ptr<hexapod_msgs::srv::SetJointState::Request> request,
      std::shared_ptr<hexapod_msgs::srv::SetJointState::Response> response) {

    std::string topic_name = "/" + prefix_ + "/" + JOINT_STATE_TOPIC;

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
