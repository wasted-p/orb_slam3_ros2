#ifndef HEXAPOD_CONTROL_REQUESTS_HPP
#define HEXAPOD_CONTROL_REQUESTS_HPP

#include "hexapod_msgs/srv/set_pose.hpp"
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_joint_state.hpp>
#include <hexapod_msgs/srv/set_marker_array.hpp>
#include <hexapod_msgs/srv/solve_ik.hpp>
#include <rclcpp/rclcpp.hpp>

void setMarkerArray(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<hexapod_msgs::srv::SetMarkerArray>::SharedPtr client,
    const std::vector<hexapod_msgs::msg::Pose> &poses, bool update = false);

void setJointState(
    rclcpp::Node::SharedPtr node,
    const rclcpp::Client<hexapod_msgs::srv::SetJointState>::SharedPtr client,
    const std::vector<std::string> &leg_names,
    const std::vector<std::vector<double>> &leg_joints);

void getPose(rclcpp::Node::SharedPtr node,
             const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
             std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response);

void solveIK(rclcpp::Node::SharedPtr node,
             const rclcpp::Client<hexapod_msgs::srv::SolveIK>::SharedPtr client,
             const std::vector<std::string> &leg_names,
             const std::vector<geometry_msgs::msg::Point> &positions,
             const std::vector<std::vector<double>> &joint_positions);

void setPose(rclcpp::Node::SharedPtr node,
             rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr client,
             const hexapod_msgs::msg::Pose &pose);
#endif // !HEXAPOD_CONTROL_REQUESTS_HPP
