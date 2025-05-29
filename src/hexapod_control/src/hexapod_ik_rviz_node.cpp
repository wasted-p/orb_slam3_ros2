#include <hexapod_control/hexapod_ik_base.hpp>

class HexapodIKRvizNode : public HexapodIKBaseNode {
  void updateJointState(std::vector<std::string> joint_names,
                        std::vector<double> joint_positions) {
    joint_state_msg_.header.frame_id = "base_footprint";

    for (size_t i = 0; i < joint_names.size(); i++) {
      joint_state_msg_.name.push_back(joint_names[i]);
      joint_state_msg_.position.push_back(joint_positions[i]);
    }

    RCLCPP_DEBUG(get_logger(),
                 "Sending Target Joint Positions = [%.2f, %.2f, %.2f]",
                 joint_positions[0], joint_positions[1], joint_positions[2]);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HexapodIKRvizNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
