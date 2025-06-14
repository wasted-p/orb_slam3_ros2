#include "hexapod_msgs/srv/execute_motion.hpp"
#include <hexapod_common/requests.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>

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

void setJointState(
    rclcpp::Node::SharedPtr node,
    const rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr client,
    const sensor_msgs::msg::JointState &joint_state) {
  auto request = std::make_shared<hexapod_msgs::srv::SetJointState::Request>();
  std::string client_id = "SetJointState Client";
  request->joint_state.name = joint_state.name;
  request->joint_state.position = joint_state.position;

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
             const std::vector<hexapod_msgs::msg::Pose> &poses,
             BatchSolveIKSuccessCallback result_callback) {

  auto request = std::make_shared<hexapod_msgs::srv::SolveIK::Request>();
  request->poses = poses;

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
          result_callback(response->joint_states);
        } else {
          RCLCPP_ERROR(rclcpp::get_logger(NAME), "SolveIK Error: %s",
                       response->message.c_str());
        }
      });
}

void sendTrajectoryGoal(
    const rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
    const std::vector<sensor_msgs::msg::JointState> &joint_states,
    const builtin_interfaces::msg::Duration &step_,
    const rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions
        options) {
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = joint_states[0].name;

  rclcpp::Duration duration(0, 0);
  rclcpp::Duration step(0, 500000000); // 0.5 sec = 500M ns
  for (const sensor_msgs::msg::JointState &joint_state : joint_states) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_state.position;
    point.time_from_start = duration;
    goal_msg.trajectory.points.push_back(point);
    duration = duration + step;
  }
  client->async_send_goal(goal_msg, options);
}

void executeMotion(
    rclcpp::Client<hexapod_msgs::srv::ExecuteMotion>::SharedPtr client,
    const std::string &name, const double &direction, const double &stride) {
  auto request = std::make_shared<hexapod_msgs::srv::ExecuteMotion::Request>();
  request->name = name;
  request->direction = direction;
  request->stride = stride;
  std::string NAME = "SetMarkerRequest";
  client->async_send_request(
      request,
      [NAME](rclcpp::Client<hexapod_msgs::srv::ExecuteMotion>::SharedFuture
                 future_response) {
        auto response = future_response.get();
        if (response->success) {
          RCLCPP_INFO(rclcpp::get_logger(NAME), "ExecuteMotion: %s",
                      response->message.c_str());
        } else {
          RCLCPP_ERROR(rclcpp::get_logger(NAME), "Adding markers failed: %s",
                       response->message.c_str());
        }
      });
}
