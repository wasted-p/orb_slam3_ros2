#ifndef HEXAPOD_COMMON_REQUESTS_HPP
#define HEXAPOD_COMMON_REQUESTS_HPP

#include "builtin_interfaces/msg/duration.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/execute_motion.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <functional>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_joint_state.hpp>
#include <hexapod_msgs/srv/set_marker_array.hpp>
#include <hexapod_msgs/srv/solve_ik.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/contexts/default_context.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

void setMarkerArray(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<hexapod_msgs::srv::SetMarkerArray>::SharedPtr client,
    const std::vector<hexapod_msgs::msg::Pose> &poses, bool update = false);

void setJointState(
    rclcpp::Node::SharedPtr node,
    const rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr client,
    const sensor_msgs::msg::JointState &joint_state);

void getPose(const rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::GetPose>::SharedPtr client,
             const std::function<void(const hexapod_msgs::msg::Pose &)>
                 success_callback);

using JointNames = std::vector<std::string>;
using JointPositions = std::vector<double>;

using SolveIKSuccessCallback =
    std::function<void(const sensor_msgs::msg::JointState &)>;
using BatchSolveIKSuccessCallback =
    std::function<void(const std::vector<sensor_msgs::msg::JointState> &)>;
void solveIK(rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedPtr client,
             const std::vector<hexapod_msgs::msg::Pose> &poses,
             BatchSolveIKSuccessCallback result_callback);

// void RCLCPP_INFO(rclcpp::Logger logger, )
void setPose(const rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr client,
             const hexapod_msgs::msg::Pose &pose, const bool relative = false);

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

void sendTrajectoryGoal(
    const rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client,
    const std::vector<sensor_msgs::msg::JointState> &joint_states,
    const double &duration,
    const rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions
        options);

void sendExecuteMotionRequest(
    rclcpp::Client<hexapod_msgs::srv::ExecuteMotion>::SharedPtr client,
    const std::string &name, const double &direction, const double &stride);
#endif // !HEXAPOD_COMMON_REQUESTS_HPP
//
