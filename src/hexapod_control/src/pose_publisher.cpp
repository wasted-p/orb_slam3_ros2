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
#include <hexapod_common/logging.hpp>
#include <hexapod_common/requests.hpp>
#include <hexapod_common/ros_constants.hpp>
#include <hexapod_common/yaml_utils.hpp>
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
  std::string prefix_;

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
    hexapod_msgs::msg::Pose pose;
    for (const auto &[leg_name, position] : initial_pose_) {
      pose.names.push_back(leg_name);
      pose.positions.push_back(position);
    }
    pose_pub_->publish(pose);
  }

  void setPose(const hexapod_msgs::msg::Pose &pose,
               const bool relative = false) {

    solveIK(
        shared_from_this(), solve_ik_client_, {pose},
        [this, pose](std::vector<sensor_msgs::msg::JointState> joint_states) {
          setJointState(shared_from_this(), set_joint_state_client_,
                        joint_states[0]);

          for (size_t i = 0; i < pose.names.size(); i++) {
            current_pose_[pose.names[i]] = pose.positions[i];
          }
          hexapod_msgs::msg::Pose msg;
          for (const auto &[leg_name, position] : current_pose_) {
            msg.names.push_back(leg_name);
            msg.positions.push_back(position);
          }

          pose_pub_->publish(msg);
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
    prefix_ = this->declare_parameter("prefix", std::string(""));

    pose_pub_ = this->create_publisher<hexapod_msgs::msg::Pose>(
        POSE_TOPIC, rclcpp::QoS(10));

    solve_ik_client_ = create_client<hexapod_msgs::srv::SolveIK>(
        joinWithSlash(prefix_, SOLVE_IK_SERVICE_NAME));

    solve_fk_client_ = create_client<hexapod_msgs::srv::SolveFK>(
        joinWithSlash(prefix_, SOLVE_FK_SERVICE_NAME));

    set_joint_state_client_ = create_client<hexapod_msgs::srv::SetJointState>(
        joinWithSlash(prefix_, SET_JOINT_STATE_SERVICE_NAME));

    get_pose_service_ = create_service<hexapod_msgs::srv::GetPose>(
        joinWithSlash(prefix_, GET_POSE_SERVICE_NAME),
        [this](
            const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
            std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response) {
          (void)request;
          response->pose = getPose();
          response->success = true;
          response->message = "Message";
        });

    set_pose_service_ = create_service<hexapod_msgs::srv::SetPose>(
        joinWithSlash(prefix_, SET_POSE_SERVICE_NAME),
        [this](
            const std::shared_ptr<hexapod_msgs::srv::SetPose::Request> request,
            std::shared_ptr<hexapod_msgs::srv::SetPose::Response> response) {
          RCLCPP_INFO(get_logger(), "NAME: %s",
                      joinWithSlash(prefix_, SET_POSE_SERVICE_NAME).c_str());
          setPose(request->pose, request->relative);
          response->success = true;
          response->message = "Successfully set pose";
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
