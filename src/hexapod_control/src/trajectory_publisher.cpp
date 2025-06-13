#include "builtin_interfaces/msg/duration.hpp"
#include "hexapod_control/requests.hpp"
#include "hexapod_control/ros_constants.hpp"
#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "hexapod_msgs/srv/set_joint_states.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <hexapod_control/requests.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node {

public:
  TrajectoryPublisher() : Node("trajectory_publisher") {
    setupROS();
    setupControllers();
  }

private:
  std::vector<std::string> controllers_;
  rclcpp::Service<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_state_service_;
  rclcpp::Service<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_states_service_;
  bool action_server_ready_ = false;

  void setupROS() {
    // Action client for joint trajectory controller
  }

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

public:
  void setupControllers() {
    // Initialize action client
    // Wait for action server
    // if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(5)))
    // {
    //   RCLCPP_ERROR(get_logger(), "Action server not available after
    //   waiting");
    // } else {
    //   RCLCPP_INFO(get_logger(), "Connected to joint trajectory controller");
    //   action_server_ready_ = true;
    // }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
