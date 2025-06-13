#include "builtin_interfaces/msg/duration.hpp"
#include "hexapod_control/requests.hpp"
#include "hexapod_control/ros_constants.hpp"
#include "hexapod_msgs/srv/set_joint_state.hpp"
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
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      trajectory_client_;
  rclcpp::Service<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_state_service_;
  bool action_server_ready_ = false;

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions
      send_goal_options_ =
          rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

  void setupROS() {

    set_joint_state_service_ = create_service<hexapod_msgs::srv::SetJointState>(
        SET_JOINT_STATE_SERVICE_NAME,
        std::bind(&TrajectoryPublisher::handleSetJointStateRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Action client for joint trajectory controller
  }

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  void handleSetJointStateRequest(
      const std::shared_ptr<hexapod_msgs::srv::SetJointState::Request> request,
      std::shared_ptr<hexapod_msgs::srv::SetJointState::Response> response) {
    RCLCPP_DEBUG(get_logger(), "SetJointState Request Recieved");
    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = request->joint_state.name;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = request->joint_state.position;
    // FIXME: Make duration variable
    point.time_from_start = rclcpp::Duration::from_seconds(1);
    goal_msg.trajectory.points.push_back(point);
    trajectory_client_->async_send_goal(goal_msg, send_goal_options_);
    response->success = true;
  }

public:
  void setupControllers() {
    send_goal_options_.result_callback =
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
        control_msgs::action::FollowJointTrajectory>(this,
                                                     TRAJECTORY_SERVICE_NAME);
    // Wait for action server
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
    } else {
      RCLCPP_INFO(get_logger(), "Connected to joint trajectory controller");
      action_server_ready_ = true;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
