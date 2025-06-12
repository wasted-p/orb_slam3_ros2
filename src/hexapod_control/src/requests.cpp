#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <functional>
#include <hexapod_control/requests.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

void setMarkerArray(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<hexapod_msgs::srv::SetMarkerArray>::SharedPtr client,
    const std::vector<hexapod_msgs::msg::Pose> &poses, bool update) {
  auto request = std::make_shared<hexapod_msgs::srv::SetMarkerArray::Request>();
  request->update = update;
  std::map<std::string, std::vector<geometry_msgs::msg::Point>> map;
  for (const hexapod_msgs::msg::Pose &pose : poses) {
    for (size_t i = 0; i < pose.names.size(); i++) {
      map[pose.names[i]].push_back(pose.positions[i]);
    }
  }

  for (const auto &entry : map) {
    hexapod_msgs::msg::PointArray trajectory;
    request->leg_names.push_back(entry.first);
    trajectory.points = entry.second;
    request->trajectories.push_back(trajectory);
  }

  client->async_send_request(
      request,
      [node](rclcpp::Client<hexapod_msgs::srv::SetMarkerArray>::SharedFuture
                 future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node->get_logger(), "%s", response->message.c_str());
        } else {
          RCLCPP_ERROR(node->get_logger(), "Adding markers failed: %s",
                       response->message.c_str());
        }
      });
};

void setJointPositions(
    rclcpp::Node::SharedPtr node,
    const rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr client,
    const std::vector<std::string> &joint_names,
    const std::vector<double> &joint_positions) {
  auto request = std::make_shared<hexapod_msgs::srv::SetJointState::Request>();
  std::string client_id = "SetJointState Client";
  request->joint_state.name = joint_names;
  request->joint_state.position = joint_positions;

  RCLCPP_INFO(node->get_logger(), "Setting Joint States");
  client->async_send_request(
      request,
      [node,
       client_id](rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedFuture
                      future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node->get_logger(), "[%s]:%s", client_id.c_str(),
                      response->message.c_str());
        } else {
          RCLCPP_ERROR(node->get_logger(), "[%s]:%s", client_id.c_str(),
                       response->message.c_str());
        }
      });
}

void getPose(rclcpp::Node::SharedPtr node,
             const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
             std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response) {
  (void)request;
  RCLCPP_DEBUG(node->get_logger(), "GetPose Request received");

  hexapod_msgs::msg::Pose pose;
  response->pose = pose;
}

// void RCLCPP_INFO(rclcpp::Logger logger, )
void setPose(const rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr client,
             const hexapod_msgs::msg::Pose &pose) {
  auto request = std::make_shared<hexapod_msgs::srv::SetPose::Request>();
  request->pose = pose;

  RCLCPP_INFO(node->get_logger(), "Sending setPose Request for Pose: %s",
              pose.name.c_str());
  client->async_send_request(
      request, [node](rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedFuture
                          future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node->get_logger(), "%s", response->message.c_str());
        } else {
          RCLCPP_ERROR(node->get_logger(), "%s", response->message.c_str());
        }
      });
}

void solveIK(rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedPtr client,
             const std::vector<std::string> &leg_names,
             const std::vector<geometry_msgs::msg::Point> &positions,
             SolveIKSuccessCallback result_callback) {

  auto request = std::make_shared<hexapod_msgs::srv::SolveIK::Request>();
  request->leg_names = leg_names;
  request->positions = positions;

  client->async_send_request(
      request, [node, result_callback](
                   rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedFuture
                       future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node->get_logger(), "%s", response->message.c_str());
          RCLCPP_INFO(node->get_logger(), "Solved Joint Positions:");
          result_callback(response->joint_names, response->joint_positions);
        } else {
          RCLCPP_ERROR(node->get_logger(), "SolveIK Error: %s",
                       response->message.c_str());
        }
      });
}
