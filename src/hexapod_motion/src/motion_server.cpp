#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hexapod_motion/motion.hpp"
#include "hexapod_msgs/msg/pose.hpp"
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
#include <tf2/LinearMath/Transform.hpp>
#include <vector>

using Poses = std::vector<hexapod_msgs::msg::Pose>;
Poses rotate(const Poses &poses, double roll, double pitch, double yaw) {
  Poses transformed_poses;

  if (poses.empty())
    return transformed_poses;

  // Store first points for each leg as pivot points
  std::map<std::string, geometry_msgs::msg::Point> pivot_points;

  // First pass: collect first points for each leg
  const auto &first_pose = poses[0];
  for (size_t leg_i = 0; leg_i < first_pose.names.size(); leg_i++) {
    const auto &leg_name = first_pose.names[leg_i];
    const auto &position = first_pose.positions[leg_i];
    pivot_points[leg_name] = position;
  }

  // Create rotation transform
  tf2::Transform rotation_transform;
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  rotation_transform.setRotation(q);
  rotation_transform.setOrigin(tf2::Vector3(0, 0, 0));

  for (auto pose : poses) {
    for (size_t leg_i = 0; leg_i < pose.names.size(); leg_i++) {
      const auto &leg_name = pose.names[leg_i];
      auto &position = pose.positions[leg_i];

      // Get pivot point for this leg
      const auto &pivot = pivot_points[leg_name];
      tf2::Vector3 pivot_vec(pivot.x, pivot.y, pivot.z);

      // Convert to tf2::Vector3
      tf2::Vector3 tf_point(position.x, position.y, position.z);

      // Translate to pivot
      tf_point -= pivot_vec;

      // Apply rotation
      tf_point = rotation_transform * tf_point;

      // Translate back
      tf_point += pivot_vec;

      // Convert back to geometry_msgs::Point
      position.x = tf_point.x();
      position.y = tf_point.y();
      position.z = tf_point.z();
    }
    transformed_poses.push_back(pose);
  }
  return transformed_poses;
}

using namespace std::chrono_literals;
class MotionServer : public rclcpp::Node {

private:
  // ROS2 Subscriptions
  std::map<std::string, Motion> motions_;
  rclcpp::Service<hexapod_msgs::srv::ExecuteMotion>::SharedPtr
      execute_motion_service_;

  bool executing_motion_;
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
          executing_motion_ = false;
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

    if (executing_motion_) {
      response->success = false;
      return;
    }
    executing_motion_ = true;
    const Motion &motion = motions_[request->name];
    RCLCPP_INFO(get_logger(), "Executing %s at angle %.4f",
                request->name.c_str(), request->direction);
    Poses transformed_poses = rotate(motion.poses, 0., 0., request->direction);
    const double duration = 0.4;
    solveIK(shared_from_this(), solve_ik_client_, transformed_poses,
            [this, duration,
             response](std::vector<sensor_msgs::msg::JointState> joint_states) {
              sendTrajectoryGoal(trajectory_client_, joint_states, duration,
                                 send_goal_options_);
              response->success = true;
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
