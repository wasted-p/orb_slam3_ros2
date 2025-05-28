#include <hexapod_control/hexapod_ik_base.hpp>
#include <rclcpp/logging.hpp>

class HexapodIKGzNode : public HexapodIKBaseNode {
  void updatePose(const hexapod_msgs::msg::Pose pose) {
    RCLCPP_INFO(get_logger(), "Recieved Pose");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HexapodIKGzNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
