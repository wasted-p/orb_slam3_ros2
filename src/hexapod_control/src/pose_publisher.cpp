#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include "hexapod_msgs/srv/solve_fk.hpp"
#include "hexapod_msgs/srv/solve_ik.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_control/hexapod.hpp>
#include <hexapod_control/requests.hpp>
#include <hexapod_control/ros_constants.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_pose.hpp>
#include <map>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/create_publisher.hpp>
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
protected:
  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Service<hexapod_msgs::srv::GetPose>::SharedPtr get_pose_service_;
  rclcpp::Service<hexapod_msgs::srv::SetPose>::SharedPtr set_pose_service_;
  rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedPtr solve_ik_client_;
  rclcpp::Client<hexapod_msgs::srv::SolveFK>::SharedPtr solve_fk_client_;
  rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr
      set_joint_state_client_;

public:
  PosePublisher() : Node("leg_control_node") {
    using namespace std::chrono_literals;
    setupROS();
  }
  ~PosePublisher() {}

private:
  std::map<std::string, geometry_msgs::msg::Point> initial_pose_;

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

  hexapod_msgs::msg::Pose toAbsolute(const hexapod_msgs::msg::Pose &pose) {
    hexapod_msgs::msg::Pose relative_pose;
    for (size_t i = 0; i < pose.names.size(); i++) {
      geometry_msgs::msg::Point position = pose.positions[i];
      const std::string &leg_name = pose.names[i];
      position.x += initial_pose_[leg_name].x;
      position.y += initial_pose_[leg_name].y;
      position.z += initial_pose_[leg_name].z;
      relative_pose.names.push_back(leg_name);
      relative_pose.positions.push_back(position);
    }
    return relative_pose;
  }

  void setPose(const hexapod_msgs::msg::Pose &incoming_pose,
               const bool relative = false) {

    hexapod_msgs::msg::Pose pose;
    if (relative)
      pose = toAbsolute(incoming_pose);
    else
      pose = incoming_pose;

    const std::vector<std::string> &leg_names = pose.names;
    const std::vector<geometry_msgs::msg::Point> &positions = pose.positions;
    solveIK(shared_from_this(), solve_ik_client_, leg_names, positions,
            [this, pose](const JointNames &joint_names,
                         const JointPositions &joint_positions) {
              setJointPositions(shared_from_this(), set_joint_state_client_,
                                joint_names, joint_positions);
              pose_pub_->publish(pose);
            });
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
          getPose(shared_from_this(), request, response);
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
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
