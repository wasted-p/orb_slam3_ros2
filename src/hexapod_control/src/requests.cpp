#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
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

  std::string NAME = "SetMarkerRequest";
  client->async_send_request(
      request,
      [NAME](rclcpp::Client<hexapod_msgs::srv::SetMarkerArray>::SharedFuture
                 future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_DEBUG(rclcpp::get_logger(NAME), "%s",
                       response->message.c_str());
        } else {
          RCLCPP_ERROR(rclcpp::get_logger(NAME), "Adding markers failed: %s",
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

  std::string NAME = "SetJointPositionsRequest";
  client->async_send_request(
      request,
      [NAME,
       client_id](rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedFuture
                      future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_DEBUG(rclcpp::get_logger(NAME), "[%s]:%s", client_id.c_str(),
                       response->message.c_str());
        } else {
          RCLCPP_ERROR(rclcpp::get_logger(NAME), "[%s]:%s", client_id.c_str(),
                       response->message.c_str());
        }
      });
}

void getPose(const rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::GetPose>::SharedPtr client,
             const std::function<void(const hexapod_msgs::msg::Pose &)>
                 success_callback) {
  auto request = std::make_shared<hexapod_msgs::srv::GetPose::Request>();

  std::string NAME = "GetPoseRequest";
  client->async_send_request(
      request, [NAME, success_callback](
                   rclcpp::Client<hexapod_msgs::srv::GetPose>::SharedFuture
                       future_response) {
        auto response = future_response.get();
        if (response->success) {
          success_callback(response->pose);
        } else {
          RCLCPP_ERROR(rclcpp::get_logger(NAME), "%s",
                       response->message.c_str());
        }
      });
}

// void RCLCPP_DEBUG(rclcpp::Logger logger, )
void setPose(const rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr client,
             const hexapod_msgs::msg::Pose &pose, const bool relative) {
  auto request = std::make_shared<hexapod_msgs::srv::SetPose::Request>();
  request->pose = pose;

  std::string NAME = "SetPoseRequest";
  client->async_send_request(
      request, [NAME](rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedFuture
                          future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_DEBUG(rclcpp::get_logger(NAME), "%s",
                       response->message.c_str());
        } else {
          RCLCPP_ERROR(rclcpp::get_logger(NAME), "%s",
                       response->message.c_str());
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

  std::string NAME = "SolveIKRequest";
  client->async_send_request(
      request, [NAME, result_callback](
                   rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedFuture
                       future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_DEBUG(rclcpp::get_logger(NAME), "%s",
                       response->message.c_str());
          RCLCPP_DEBUG(rclcpp::get_logger(NAME), "Solved Joint Positions:");
          result_callback(response->joint_names, response->joint_positions);
        } else {
          RCLCPP_ERROR(rclcpp::get_logger(NAME), "SolveIK Error: %s",
                       response->message.c_str());
        }
      });
}
