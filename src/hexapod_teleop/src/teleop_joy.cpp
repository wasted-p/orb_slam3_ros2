#include "hexapod_msgs/srv/execute_motion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

    // Create subscription for joystick data
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, 10,
        std::bind(&TeleopJoy::joystickInputCallback, this,
                  std::placeholders::_1));
  }

  void joystickInputCallback(const sensor_msgs::msg::Joy &msg) {
    RCLCPP_DEBUG(get_logger(),
                 "Controller = [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                 msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3],
                 msg.axes[4], msg.axes[5], msg.axes[6], msg.axes[7]);
    // controller->setControllerState(msg.axes);
    // hexapod_msgs::msg::Motion action_msg;
    const float default_axes[8] = {0, 0, 1, 0, 0, 1, 0, 0};

    bool idle = std::equal(msg.axes.begin(), msg.axes.end(), default_axes);
    const std::string name = "tripod";
    // sendExecuteMotionRequest(execute_motion_client_, name, 0, 1);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleopJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
