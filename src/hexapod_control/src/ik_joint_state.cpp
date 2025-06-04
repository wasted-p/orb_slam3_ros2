#include <hexapod_control/ik_base.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class HexapodIKRvizNode : public HexapodIKBaseNode {

public:
  HexapodIKRvizNode() { setupROS(); }

private:
  sensor_msgs::msg::JointState joint_state_msg_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;

  void setupROS() {
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                             rclcpp::QoS(10));
  }

  void updateJointState(std::vector<std::string> joint_names,
                        std::vector<double> joint_positions) {
    joint_state_msg_.header.frame_id = "base_footprint";

    joint_state_msg_.name = {};
    joint_state_msg_.position = {};

    RCLCPP_INFO(get_logger(), "Recieved JointState");
    for (size_t i = 0; i < joint_names.size(); i++) {
      RCLCPP_INFO(get_logger(), "%s=%.4f", joint_names[i].c_str(),
                  joint_positions[i]);
      joint_state_msg_.name.push_back(joint_names[i]);
      joint_state_msg_.position.push_back(joint_positions[i]);
    }
  }

  void timerCallback() {
    joint_state_msg_.header.stamp = get_clock()->now();
    joint_state_publisher_->publish(joint_state_msg_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HexapodIKRvizNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
