#include "hexapod_msgs/srv/execute_motion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <hexapod_common/requests.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_common/yaml_utils.hpp>
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
  TeleopJoy() : Node("teleop_joy") { setupROS(); }

private:
  std::string prefix_;
  void setupROS() {

    prefix_ = this->declare_parameter("prefix", std::string(""));
    execute_motion_client_ = create_client<hexapod_msgs::srv::ExecuteMotion>(
        joinWithSlash(prefix_, EXECUTE_MOTION_SERVICE_NAME));

    // Create subscription for joystick data
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, 10,
        std::bind(&TeleopJoy::joystickInputCallback, this,
                  std::placeholders::_1));
  }

  void joystickInputCallback(const sensor_msgs::msg::Joy &msg) {
    // Left stick input
    float x = msg.axes[0]; // left/right (inverted)
    float y = msg.axes[1]; // forward/backward

    if (std::abs(x) < 0.05 && std::abs(y) < 0.05) {
      return; // stick is idle
    }

    // Compute angle from north (positive Y axis)
    double angle_rad = std::atan2(-x, -y); // note: atan2(x, y) not y, x

    RCLCPP_DEBUG(get_logger(),
                 "Controller = [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                 msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3],
                 msg.axes[4], msg.axes[5], msg.axes[6], msg.axes[7]);
    RCLCPP_DEBUG(get_logger(), "Left stick angle: %.2f rad", angle_rad);

    const std::string name = "tripod";
    sendExecuteMotionRequest(execute_motion_client_, name, angle_rad, 1);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
