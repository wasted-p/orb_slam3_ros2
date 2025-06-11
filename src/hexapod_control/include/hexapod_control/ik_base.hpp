#ifndef HEXAPOD_IK_BASE_HPP
#define HEXAPOD_IK_BASE_HPP

// TODO:
// - [ ] Add Context menu
// - [ ] Add Buttons to markers
// - [ ] Try other IK algorithms
// - [ ] Make algorithm more performant and accurate
// - [ ] Show Trail in Rviz for legs
// - [ ] Show reachable regions
// - [ ] Give each control a different color
// - [ ] Dont show control when joint is not active
// - [ ] Fix TF and RobotModel warning
//
// FIXME: Remove unneeded includes
// NOTE: Use most performant and appropriate algorithms
// #include "hexapod_control/msg/show_marker.hpp"
#include "6_dof_marker.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_control/planning_group.hpp>
#include <hexapod_msgs/msg/command.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class HexapodIKBaseNode : public rclcpp::Node {
protected:
  const static int CHAIN_COUNT = 6;
  std::map<std::string, KDL::Chain> chains_;
  KDL::Tree kdl_tree_;
  std::string urdf_string;
  bool robot_description_loaded_ = false;
  PlanningGroup planning_group;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<hexapod_msgs::msg::Command>::SharedPtr command_sub_;
  rclcpp::Service<hexapod_msgs::srv::GetPose>::SharedPtr get_pose_service_;
  rclcpp::Service<hexapod_msgs::srv::SetPose>::SharedPtr set_pose_service_;
  hexapod_msgs::msg::Pose current_pose_;
  // hexapod_msgs::msg::Pose pose_msg_;
  std::map<std::string, geometry_msgs::msg::Point> pose_msgs_;
  sensor_msgs::msg::JointState joint_state_msg_;

  std::string LEG_NAMES[6] = {
      "top_left",  "mid_left",  "bottom_left",
      "top_right", "mid_right", "bottom_right",
  };

public:
  HexapodIKBaseNode();
  ~HexapodIKBaseNode();

protected:
  virtual void updateJointState(std::vector<std::string> joint_names,
                                std::vector<double> joint_positions,
                                builtin_interfaces::msg::Duration duration) = 0;

  void setupROS();
  virtual void timerCallback() = 0;

private:
  void poseUpdateCallback(const hexapod_msgs::msg::Pose pose);
  void command_callback(const hexapod_msgs::msg::Command command);
  void updatePose(const hexapod_msgs::msg::Pose pose);
  void readRobotDescription();
  void setupControl(std::string leg_name, geometry_msgs::msg::Point pos);
  std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f);
  void setupKDL();
  void handleGetPoseRequest(
      const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
      std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response);
  TranslationMarker add6DofControl();
  void processFeedback(
      std::string leg_name,
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback);
};

#endif
