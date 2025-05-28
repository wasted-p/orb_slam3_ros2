#include <control_msgs/action/follow_joint_trajectory.hpp>
// #include <control_msgs/msg/follow_joint_trajectory>
#include <hexapod_control/hexapod_ik_base.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class HexapodIKGzNode : public HexapodIKBaseNode {
public:
  HexapodIKGzNode() { setupControllers(); }

private:
  // Action client for joint trajectory controller
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      trajectory_client_;

  // Joint names for all legs (adjust based on your URDF)
  std::vector<std::string> joint_names_ = {
      // Left legs
      "top_left_coxa_joint", "top_left_femur_joint", "top_left_tibia_joint",
      "mid_left_coxa_joint", "mid_left_femur_joint", "mid_left_tibia_joint",
      "bottom_left_coxa_joint", "bottom_left_femur_joint",
      "bottom_left_tibia_joint",
      // Right legs
      "top_right_coxa_joint", "top_right_femur_joint", "top_right_tibia_joint",
      "mid_right_coxa_joint", "mid_right_femur_joint", "mid_right_tibia_joint",
      "bottom_right_coxa_joint", "bottom_right_femur_joint",
      "bottom_right_tibia_joint"};

public:
  void updatePose(const hexapod_msgs::msg::Pose pose) {
    RCLCPP_INFO(this->get_logger(), "Recieved Pose %s", pose.name.c_str());
  }

  void setupControllers() {
    // Initialize action client
    trajectory_client_ = rclcpp_action::create_client<
        control_msgs::action::FollowJointTrajectory>(
        this, "/legs_joint_trajectory_controller/follow_joint_trajectory");

    // Wait for action server
    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
    } else {
      RCLCPP_INFO(get_logger(), "Connected to joint trajectory controller");
    }
  }

  void sendJointTrajectory(const std::vector<double> &joint_positions) {
    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();

    // Set joint names
    goal_msg.trajectory.joint_names = joint_names_;

    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.velocities.resize(joint_names_.size(), 0.0);
    point.time_from_start =
        rclcpp::Duration::from_seconds(2.0); // 2 second duration

    // Add point to trajectory
    goal_msg.trajectory.points.push_back(point);

    // Set trajectory header
    goal_msg.trajectory.header.stamp = this->get_clock()->now();

    // Send goal
    auto send_goal_options = rclcpp_action::Client<
        control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<
                   control_msgs::action::FollowJointTrajectory>>
                   goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
          } else {
            RCLCPP_INFO(get_logger(),
                        "Goal accepted by server, waiting for result");
          }
        };

    send_goal_options.feedback_callback =
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<
                   control_msgs::action::FollowJointTrajectory>>,
               const std::shared_ptr<
                   const control_msgs::action::FollowJointTrajectory::Feedback>
                   feedback) {
          // Handle feedback if needed
          (void)feedback; // Suppress unused parameter warning
        };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<
               control_msgs::action::FollowJointTrajectory>::WrappedResult
                   &result) {
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Trajectory execution succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Trajectory execution was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Trajectory execution was canceled");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
          }
        };

    RCLCPP_INFO(get_logger(), "Sending trajectory with %zu joints",
                joint_names_.size());
    trajectory_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HexapodIKGzNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
