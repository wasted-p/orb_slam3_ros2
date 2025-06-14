#include "hexapod_msgs/srv/execute_motion.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <hexapod_common/requests.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_msgs/srv/execute_motion.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

using namespace std::chrono_literals;
class TeleopJoy : public rclcpp::Node {

private:
  // ROS2 Subscriptions
  rclcpp::Client<hexapod_msgs::srv::ExecuteMotion>::SharedPtr
      execute_motion_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

public:
  TeleopJoy() : Node("Joy To Motion") { setupROS(); }

private:
  void setupROS() {
    execute_motion_client_ = create_client<hexapod_msgs::srv::ExecuteMotion>(
        EXECUTE_MOTION_SERVICE_NAME);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
