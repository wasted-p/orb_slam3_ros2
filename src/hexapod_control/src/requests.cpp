#include "hexapod_msgs/srv/set_pose.hpp"
#include <hexapod_control/requests.hpp>

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

void getPose(rclcpp::Node::SharedPtr node,
             const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
             std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response) {
  (void)request;
  RCLCPP_DEBUG(node->get_logger(), "GetPose Request received");

  hexapod_msgs::msg::Pose pose;
  response->pose = pose;
}

void setPose(rclcpp::Node::SharedPtr node,
             rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr client,
             const hexapod_msgs::msg::Pose &pose) {
  auto request = std::make_shared<hexapod_msgs::srv::SetPose::Request>();
  request->pose = pose;

  client->async_send_request(
      request, [node](rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedFuture
                          future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(node->get_logger(), "Successfully sent SetPose request");
        } else {

          RCLCPP_ERROR(node->get_logger(), "Error in SetPose request");
        }
      });
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
