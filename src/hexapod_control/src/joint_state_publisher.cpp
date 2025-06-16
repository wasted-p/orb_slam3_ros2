#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <hexapod_common/hexapod.hpp>
#include <hexapod_common/requests.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_common/yaml_utils.hpp>
#include <hexapod_msgs/srv/set_joint_state.hpp>
#include <map>
#include <rclcpp/client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

using namespace std::chrono_literals;

class JointStatePublisher : public rclcpp::Node {
public:
  JointStatePublisher() : Node("joint_state_publisher") { setupROS(); }
  ~JointStatePublisher() {};

private:
  std::string prefix_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Service<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_state_service_;
  sensor_msgs::msg::JointState joint_state_msg_;
  std::map<std::string, double> initial_positions_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions
      send_goal_options_ = rclcpp_action::Client<
          control_msgs::action::FollowJointTrajectory>::SendGoalOptions();

  void timerCallback() {
    joint_state_msg_.header.stamp = get_clock()->now();
    joint_state_publisher_->publish(joint_state_msg_);
  }

  void setupROS() {

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

    // Declare and get the YAML string parameter
    std::string yaml_string = this->declare_parameter(
        "initial_positions", std::string("{initial_positions:}"));

    parseYamls(yaml_string, initial_positions_);

    // Declare and get the YAML string parameter
    prefix_ = this->declare_parameter("prefix", std::string(""));
    // timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100),
    //     std::bind(&JointStatePublisher::timerCallback, this));

    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>(
            joinWithSlash(prefix_, JOINT_STATE_TOPIC), rclcpp::QoS(10));

    set_joint_state_service_ = create_service<hexapod_msgs::srv::SetJointState>(
        joinWithSlash(prefix_, SET_JOINT_STATE_SERVICE_NAME),
        std::bind(&JointStatePublisher::handleSetJointStateRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    for (const auto &entry : initial_positions_) {
      joint_state_msg_.name.push_back(entry.first);
      joint_state_msg_.position.push_back(entry.second);
    }

    joint_state_msg_.header.frame_id = "base_footprint";

    trajectory_client_ = rclcpp_action::create_client<
        control_msgs::action::FollowJointTrajectory>(
        this, joinWithSlash(prefix_, TRAJECTORY_SERVICE_NAME));
  }

  void handleSetJointStateRequest(
      const std::shared_ptr<hexapod_msgs::srv::SetJointState::Request> request,
      std::shared_ptr<hexapod_msgs::srv::SetJointState::Response> response) {
    sendTrajectoryGoal(trajectory_client_, {request->joint_state}, 0.5,
                       send_goal_options_);

    joint_state_msg_ = request->joint_state;
    joint_state_msg_.header.stamp = this->now();
    response->success = true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
