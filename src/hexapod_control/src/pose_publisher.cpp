#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include "hexapod_msgs/srv/solve_fk.hpp"
#include "hexapod_msgs/srv/solve_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_common/hexapod.hpp>
#include <hexapod_common/requests.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_pose.hpp>
#include <map>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

void parseYaml(std::string &yaml_string,
               std::map<std::string, geometry_msgs::msg::Point> map) {
  try {
    YAML::Node root = YAML::Load(yaml_string);
    for (const auto &pair : root["initial_pose"]) {
      std::string joint_name = pair.first.as<std::string>();
      geometry_msgs::msg::Point position;
      position.x = pair.second["x"].as<double>();
      position.y = pair.second["y"].as<double>();
      position.z = pair.second["z"].as<double>();
      map[joint_name] = position;
    }
  } catch (const YAML::Exception &e) {
    throw std::runtime_error(e.what());
  }
}

class PosePublisher : public rclcpp::Node {
private:
  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Service<hexapod_msgs::srv::GetPose>::SharedPtr get_pose_service_;
  rclcpp::Service<hexapod_msgs::srv::SetPose>::SharedPtr set_pose_service_;
  rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedPtr solve_ik_client_;
  rclcpp::Client<hexapod_msgs::srv::SolveFK>::SharedPtr solve_fk_client_;
  rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_state_client_;
  std::map<std::string, geometry_msgs::msg::Point> initial_pose_;
  std::map<std::string, geometry_msgs::msg::Point> current_pose_;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      trajectory_client_;

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions
      send_goal_options_ =
          rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

public:
  PosePublisher() : Node("leg_control_node") {
    using namespace std::chrono_literals;
    setupROS();
    getInitialPose();
    current_pose_ = initial_pose_;
  }
  ~PosePublisher() {}

private:
  void getInitialPose() {
    // Declare and get the YAML string parameter
    std::string yaml_string =
        this->declare_parameter("initial_pose", std::string("{initial_pose:}"));

    parseYaml(yaml_string, initial_pose_);
    RCLCPP_INFO(this->get_logger(), "Loaded Initial Pose:");
    for (const auto &entry : initial_pose_) {
      RCLCPP_INFO(this->get_logger(), " - %s=[%.4f,%.4f,%.4f]",
                  entry.first.c_str(), entry.second.x, entry.second.y,
                  entry.second.z);
    }
  }

  void setPose(const hexapod_msgs::msg::Pose &pose,
               const bool relative = false) {

    solveIK(
        shared_from_this(), solve_ik_client_, {pose},
        [this, pose](std::vector<sensor_msgs::msg::JointState> joint_states) {
          setJointState(shared_from_this(), set_joint_state_client_,
                        joint_states[0]);
          rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.5);
          sendTrajectoryGoal(trajectory_client_, joint_states, duration,
                             send_goal_options_);

          for (size_t i = 0; i < pose.names.size(); i++) {
            current_pose_[pose.names[i]] = pose.positions[i];
          }
          pose_pub_->publish(pose);
        });
  }

  hexapod_msgs::msg::Pose getPose() {
    hexapod_msgs::msg::Pose pose;
    for (const auto &entry : current_pose_) {
      pose.names.push_back(entry.first);
      pose.positions.push_back(entry.second);
    }
    return pose;
  }

  void setupROS() {
    pose_pub_ = this->create_publisher<hexapod_msgs::msg::Pose>(
        POSE_TOPIC, rclcpp::QoS(10));

    solve_ik_client_ =
        create_client<hexapod_msgs::srv::SolveIK>(SOLVE_IK_SERVICE_NAME);

    solve_fk_client_ =
        create_client<hexapod_msgs::srv::SolveFK>(SOLVE_FK_SERVICE_NAME);

    set_joint_state_client_ = create_client<hexapod_msgs::srv::SetJointState>(
        SET_JOINT_STATE_SERVICE_NAME);

    get_pose_service_ = create_service<hexapod_msgs::srv::GetPose>(
        GET_POSE_SERVICE_NAME,
        [this](
            const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
            std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response) {
          (void)request;
          response->pose = getPose();
          response->success = true;
          response->message = "Message";
        });

    set_pose_service_ = create_service<hexapod_msgs::srv::SetPose>(
        SET_POSE_SERVICE_NAME,
        [this](
            const std::shared_ptr<hexapod_msgs::srv::SetPose::Request> request,
            std::shared_ptr<hexapod_msgs::srv::SetPose::Response> response) {
          setPose(request->pose, request->relative);
          response->success = true;
          response->message = "Successfully set pose";
        });

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
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
