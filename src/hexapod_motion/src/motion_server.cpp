#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hexapod_msgs/srv/solve_ik.hpp"
#include <cmath>
#include <hexapod_control/requests.hpp>
#include <hexapod_control/ros_constants.hpp>
#include <hexapod_motion/motion.hpp>
#include <hexapod_motion/utils.hpp>
#include <hexapod_msgs/srv/execute_motion.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>

using namespace std::chrono_literals;
class MotionServer : public rclcpp::Node {

private:
  // ROS2 Subscriptions
  std::map<std::string, Motion> motions_;
  rclcpp::Service<hexapod_msgs::srv::ExecuteMotion>::SharedPtr
      execute_motion_service_;
  rclcpp::Service<hexapod_msgs::srv::SolveIK>::SharedPtr solve_ik_service_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      trajectory_client_;

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions
      send_goal_options_ =
          rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

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
            &MotionServer::handleExecuteMotionRequest);

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
  }

  void handleExecuteMotionRequest(
      const std::shared_ptr<hexapod_msgs::srv::ExecuteMotion::Request> request,
      std::shared_ptr<hexapod_msgs::srv::ExecuteMotion::Response> response) {
    const Motion &motion = motions_["tripod"];
    // solveIK(shared_from_this(), solve_ik_service_, pose.names,
    // pose.positions,
    //         [this](const JointNames &joint_names,
    //                const JointPositions &joint_positions) {
    //           rclcpp::Duration duration =
    //           rclcpp::Duration::from_seconds(0.5);
    //           sendTrajectoryGoal(trajectory_client_, joint_names,
    //                              joint_positions, duration,
    //                              send_goal_options_);
    //         });
  };
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
