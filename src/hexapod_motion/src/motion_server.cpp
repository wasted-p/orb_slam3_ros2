#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hexapod_motion/motion.hpp"
#include "hexapod_msgs/srv/execute_motion.hpp"
#include "hexapod_msgs/srv/solve_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <hexapod_common/requests.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_motion/utils.hpp>
#include <hexapod_msgs/srv/execute_motion.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/client.hpp>
#include <vector>

using namespace std::chrono_literals;
class MotionServer : public rclcpp::Node {

private:
  // ROS2 Subscriptions
  std::map<std::string, Motion> motions_;
  rclcpp::Service<hexapod_msgs::srv::ExecuteMotion>::SharedPtr
      execute_motion_service_;

  rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedPtr solve_ik_client_;
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

    solve_ik_client_ =
        create_client<hexapod_msgs::srv::SolveIK>(SOLVE_IK_SERVICE_NAME);

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

    trajectory_client_ = rclcpp_action::create_client<
        control_msgs::action::FollowJointTrajectory>(this,
                                                     TRAJECTORY_SERVICE_NAME);

    execute_motion_service_ = create_service<hexapod_msgs::srv::ExecuteMotion>(
        EXECUTE_MOTION_SERVICE_NAME,
        std::bind(&MotionServer::handleExecuteMotionRequest, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  void handleExecuteMotionRequest(
      const hexapod_msgs::srv::ExecuteMotion::Request::SharedPtr request,
      hexapod_msgs::srv::ExecuteMotion::Response::SharedPtr response) {

    std::string name = "tripod";
    const Motion &motion = motions_[name];
    RCLCPP_INFO(get_logger(), "Executing %s", name.c_str());
    solveIK(shared_from_this(), solve_ik_client_, motion.poses,
            [this](std::vector<sensor_msgs::msg::JointState> joint_states) {
              rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.5);
              sendTrajectoryGoal(trajectory_client_, joint_states, duration,
                                 send_goal_options_);
            });
  };
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
