#include "builtin_interfaces/msg/duration.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <hexapod_control/ik_base.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using namespace std::chrono_literals;

class IkTrajectoryNode : public HexapodIKBaseNode {

public:
  IkTrajectoryNode() { setupControllers(); }

private:
  std::vector<std::string> controllers_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      trajectory_client_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  bool action_server_ready_ = false;

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions
      send_goal_options =
          rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  // Action client for joint trajectory controller

  void updateJointState(std::vector<std::string> joint_names,
                        std::vector<double> joint_positions,
                        builtin_interfaces::msg::Duration duration) {

    RCLCPP_DEBUG(get_logger(), "Recieved new pose command:");

    for (size_t i = 0; i < joint_names.size(); i++) {
      RCLCPP_DEBUG(get_logger(), " - %s=[%.4f]", joint_names[i].c_str(),
                   joint_positions[i]);
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.time_from_start = duration;
    goal_msg.trajectory.points.push_back(point);

    trajectory_client_->async_send_goal(goal_msg, send_goal_options);
  }

protected:
  void timerCallback() {}

public:
  void checkActionServer() {
    if (!action_server_ready_ && trajectory_client_->action_server_is_ready()) {
      action_server_ready_ = true;
      wait_timer_->cancel();
      RCLCPP_INFO(get_logger(), "Connected to joint trajectory controller");
    }
  }

  void setupControllers() {
    send_goal_options.result_callback =
        [this](const GoalHandle::WrappedResult &result) {
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_DEBUG(this->get_logger(), "Trajectory execution succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Trajectory execution aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Trajectory execution canceled");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
          }
        };
    // Initialize action client
    trajectory_client_ = rclcpp_action::create_client<
        control_msgs::action::FollowJointTrajectory>(
        this, "/legs_joint_trajectory_controller/follow_joint_trajectory");

    // Wait for action server
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
    } else {
      RCLCPP_INFO(get_logger(), "Connected to joint trajectory controller");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IkTrajectoryNode>();

  rclcpp::spin(node);
  // Register shutdown callback to safely teardown before rclcpp::shutdown
  rclcpp::shutdown();
  return 0;
}
