#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "hexapod_msgs/srv/set_joint_state.hpp"
#include "hexapod_msgs/srv/set_marker_array.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include "hexapod_msgs/srv/solve_fk.hpp"
#include "hexapod_msgs/srv/solve_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_control/hexapod.hpp>
#include <hexapod_control/ros_constants.hpp>
#include <hexapod_msgs/msg/command.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_pose.hpp>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <vector>

void handleGetPoseRequest(
    rclcpp::Node::SharedPtr node,
    const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
    std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response) {
  (void)request;
  RCLCPP_DEBUG(node->get_logger(), "GetPose Request received");

  hexapod_msgs::msg::Pose pose;
  response->pose = pose;
}

void setMarkerArray(
    rclcpp::Node::SharedPtr node,
    const rclcpp::Client<hexapod_msgs::srv::SetMarkerArray>::SharedPtr client,
    const std::vector<std::string> &leg_names,
    const std::vector<geometry_msgs::msg::Point> &positions) {

  auto request = std::make_shared<hexapod_msgs::srv::SetMarkerArray::Request>();
  request->leg_names = leg_names;
  request->positions = positions;
  client->async_send_request(
      request,
      [node](rclcpp::Client<hexapod_msgs::srv::SetMarkerArray>::SharedFuture
                 future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node->get_logger(),
                      "Successfully sent SetMarkerArray request");
        } else {
          RCLCPP_ERROR(node->get_logger(),
                       "Error in SetMarkerArray request: %s",
                       response->message.c_str());
        }
      });
}

void setJointState(
    rclcpp::Node::SharedPtr node,
    const rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr client,
    const std::vector<std::string> &leg_names,
    const std::vector<std::vector<double>> &leg_joints) {
  auto request = std::make_shared<hexapod_msgs::srv::SetJointState::Request>();
  sensor_msgs::msg::JointState js;
  for (size_t i = 0; i < leg_names.size(); i++) {
    const std::string &leg_name = leg_names[i];
    js.name.insert(js.name.cbegin(),
                   {leg_name + "_rotate_joint", leg_name + "_abduct_joint",
                    leg_name + "_retract_joint"});
    js.position.insert(js.position.cbegin(),
                       {leg_joints[i][0], leg_joints[i][1], leg_joints[i][2]});
  }
}

void solveIK(rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedPtr client,
             const std::vector<std::string> &leg_names,
             const std::vector<geometry_msgs::msg::Point> &positions,
             const std::vector<std::vector<double>> &joint_positions) {

  auto request = std::make_shared<hexapod_msgs::srv::SolveIK::Request>();
  request->leg_names = leg_names;
  request->positions = positions;
  client->async_send_request(
      request, [node](rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedFuture
                          future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node->get_logger(), "Successfully sent SetPose request");
        } else {
          RCLCPP_ERROR(node->get_logger(), "Error in SetPose request");
        }
      });
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
  void setPose(const hexapod_msgs::msg::Pose pose) {
    RCLCPP_INFO(get_logger(), "Updating Pose=%s", pose.name.c_str());

    std::vector<std::vector<double>> joint_positions;
    std::vector<geometry_msgs::msg::Point> positions = pose.positions;
    solveIK(shared_from_this(), solve_ik_client_, pose.names, pose.positions,
            joint_positions);
    setJointState(shared_from_this(), set_joint_state_client_, pose.names,
                  joint_positions);
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
          handleGetPoseRequest(shared_from_this(), request, response);
        });

    set_pose_service_ = create_service<hexapod_msgs::srv::SetPose>(
        SET_POSE_SERVICE_NAME,
        [this](
            const std::shared_ptr<hexapod_msgs::srv::SetPose::Request> request,
            std::shared_ptr<hexapod_msgs::srv::SetPose::Response> response) {
          hexapod_msgs::msg::Pose &pose = request->pose;

          for (size_t i = 0; i < pose.names.size(); i++) {
            RCLCPP_INFO(get_logger(), " - %s = [%.4f,%.4f,%.4f]",
                        pose.names[i].c_str(), pose.positions[i].x,
                        pose.positions[i].y, pose.positions[i].z);
          }
          setPose(request->pose);
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
