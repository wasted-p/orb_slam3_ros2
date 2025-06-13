#include <cmath>
#include <hexapod_control/ros_constants.hpp>
#include <hexapod_motion/motion.hpp>
#include <hexapod_motion/utils.hpp>
#include <hexapod_msgs/srv/execute_motion.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

using namespace std::chrono_literals;
class MotionServer : public rclcpp::Node {

private:
  // ROS2 Subscriptions
  std::map<std::string, Motion> motions_;
  rclcpp::Service<hexapod_msgs::srv::ExecuteMotion>::SharedPtr
      execute_motion_service_;

public:
  MotionServer() : Node("action_server_node") {
    setupROS();
    loadFromYaml(motions_);
  }

private:
  void setupROS() {
    execute_motion_service_ =
        this->create_service<hexapod_msgs::srv::ExecuteMotion>(
            EXECUTE_MOTION_SERVICE_NAME,
            [this](
                const std::shared_ptr<hexapod_msgs::srv::ExecuteMotion::Request>
                    request,
                std::shared_ptr<hexapod_msgs::srv::ExecuteMotion::Response>
                    response) {});
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
