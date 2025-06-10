#include "builtin_interfaces/msg/duration.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include <hexapod_control/ik_base.hpp>

HexapodIKBaseNode::~HexapodIKBaseNode() {}

void HexapodIKBaseNode::updatePose(const hexapod_msgs::msg::Pose pose) {
  std::vector<double> solved;
  std::string leg_name;
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Pose int_marker_pose;

  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  builtin_interfaces::msg::Duration duration = pose.duration;
  for (size_t i = 0; i < pose.names.size(); i++) {
    leg_name = pose.names[i];
    position = pose.positions[i];
    RCLCPP_DEBUG(get_logger(),
                 "Recieved Leg Pose Message for leg %s with positions=[%.2f, "
                 "%.2f, %.2f]",
                 leg_name.c_str(), position.x, position.y, position.z);

    int status = planning_group.calculateJntArray(chains_.at(leg_name),
                                                  position, solved);

    if (status < 0 || solved.size() < 3) {
      RCLCPP_ERROR(get_logger(),
                   "IK failed or returned insufficient joint positions for leg "
                   "%s (status: %d, size: %zu)",
                   leg_name.c_str(), status, solved.size());
      return;
    }

    joint_names.push_back(leg_name + "_rotate_joint");
    joint_names.push_back(leg_name + "_abduct_joint");
    joint_names.push_back(leg_name + "_retract_joint");

    joint_positions.push_back(solved[0]);
    joint_positions.push_back(solved[1]);
    joint_positions.push_back(solved[2]);

    int_marker_pose.position = position;
    server_->setPose(leg_name, int_marker_pose);
    server_->applyChanges();
  }

  joint_state_msg_.name = joint_names;
  joint_state_msg_.position = joint_positions;

  updateJointState(joint_names, joint_positions, duration);
}

HexapodIKBaseNode::HexapodIKBaseNode() : Node("leg_control_node") {
  using namespace std::chrono_literals;

  readRobotDescription();
  setupROS();
  setupKDL();
}
void HexapodIKBaseNode::setupKDL() {
  hexapod_msgs::msg::Pose pose;
  geometry_msgs::msg::Point pos;
  for (std::string leg_name : LEG_NAMES) {
    KDL::Chain chain;
    kdl_tree_.getChain("base_footprint", leg_name + "_foot", chain);
    chains_.insert({leg_name, chain});

    pos = planning_group.calculatePosition(chains_.at(leg_name), {0, 0, 0});

    setupControl(leg_name, pos);

    pose.names.push_back(leg_name);
    pose.positions.push_back(pos);

    RCLCPP_DEBUG(get_logger(), "Setting %s to [%.4f,%.4f,%.4f]",
                 leg_name.c_str(), pos.x, pos.y, pos.z);
  };
  pose_pub_->publish(pose);
}

void HexapodIKBaseNode::poseUpdateCallback(const hexapod_msgs::msg::Pose pose) {
  RCLCPP_DEBUG(get_logger(), "Recieved Pose:");
  for (size_t i = 0; i < pose.names.size(); ++i) {
    const std::string &leg_name = pose.names[i];
    const auto &position = pose.positions[i];

    RCLCPP_DEBUG(get_logger(), " - [%s]: [%.4f, %.4f, %.4f]", leg_name.c_str(),
                 position.x, position.y, position.z);

    pose_msgs_[leg_name] = position; // Update or insert
  }
  updatePose(pose);
}

void HexapodIKBaseNode::command_callback(
    const hexapod_msgs::msg::Command command) {
  RCLCPP_DEBUG(get_logger(), "Command %s", command.command_type.c_str());
}

void HexapodIKBaseNode::readRobotDescription() {
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "robot_state_publisher");

  auto future = parameters_client->get_parameters({"robot_description"});

  // Wait for parameter with timeout
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future, std::chrono::seconds(10)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto parameters = future.get();
    if (!parameters.empty() && !parameters[0].as_string().empty()) {
      urdf_string = parameters[0].as_string();
      RCLCPP_DEBUG(this->get_logger(),
                   "Robot description obtained from robot_state_publisher");
      if (kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
        robot_description_loaded_ = true;
        RCLCPP_DEBUG(this->get_logger(), "Parsed KDL Tree from URDF");
      }
    }
  }
}

void HexapodIKBaseNode::setupControl(std::string leg_name,
                                     geometry_msgs::msg::Point pos) {
  RCLCPP_DEBUG(get_logger(), "Setting up %s Interactive Control with joints %d",
               leg_name.c_str(), chains_.at(leg_name).getNrOfJoints());

  TranslationMarker marker = add6DofControl();
  marker.pose.position = pos;
  marker.name = leg_name;
  marker.header.frame_id = "base_footprint";
  server_->insert(marker);
  server_->setCallback(
      marker.name,
      [this,
       leg_name](const visualization_msgs::msg::InteractiveMarkerFeedback::
                     ConstSharedPtr &feedback) {
        processFeedback(leg_name, feedback);
      });

  // Apply changes to server
  server_->applyChanges();
}

std_msgs::msg::ColorRGBA HexapodIKBaseNode::makeColor(float r, float g, float b,
                                                      float a) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

void HexapodIKBaseNode::setupROS() {

  std::string POSE_TOPIC = "/hexapod/pose";
  pose_sub_ = this->create_subscription<hexapod_msgs::msg::Pose>(
      POSE_TOPIC,
      10, // QoS history depth
      std::bind(&HexapodIKBaseNode::poseUpdateCallback, this,
                std::placeholders::_1));
  pose_pub_ = this->create_publisher<hexapod_msgs::msg::Pose>(POSE_TOPIC,
                                                              rclcpp::QoS(10));

  command_sub_ = this->create_subscription<hexapod_msgs::msg::Command>(
      "hexapod_control/command", // topic name
      10,                        // QoS history depth
      std::bind(&HexapodIKBaseNode::command_callback, this,
                std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HexapodIKBaseNode::timerCallback, this));

  service_ = create_service<hexapod_msgs::srv::GetPose>(
      "control/pose", std::bind(&HexapodIKBaseNode::handleGetPoseRequest, this,
                                std::placeholders::_1, std::placeholders::_2));

  joint_state_msg_.header.frame_id = "base_footprint";

  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "interactive_marker_server", this, rclcpp::QoS(10), rclcpp::QoS(10));
}

void HexapodIKBaseNode::handleGetPoseRequest(
    const std::shared_ptr<hexapod_msgs::srv::GetPose::Request> request,
    std::shared_ptr<hexapod_msgs::srv::GetPose::Response> response) {
  RCLCPP_DEBUG(get_logger(), "GetPose Request received");

  hexapod_msgs::msg::Pose pose;

  for (const auto &entry : pose_msgs_) {
    pose.names.push_back(entry.first);
    pose.positions.push_back(entry.second);

    RCLCPP_DEBUG(get_logger(), " - Leg '%s' = [%.2f, %.2f, %.2f]",
                 entry.first.c_str(), entry.second.x, entry.second.y,
                 entry.second.z);
  }

  // Fill response
  response->pose = pose;
  response->joint_state = joint_state_msg_;
}

TranslationMarker HexapodIKBaseNode::add6DofControl() {
  TranslationMarker marker;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  // Adding Menu Items
  marker.menu_handler.insert("Reset");
  // Insert marker and set feedback callback
  return marker;
}

void HexapodIKBaseNode::processFeedback(
    std::string leg_name,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {

  switch (feedback->event_type) {
  case InteractiveMarkerFeedback::BUTTON_CLICK:
    RCLCPP_DEBUG(this->get_logger(), "Button Clicked");
    break;

  case InteractiveMarkerFeedback::MENU_SELECT:
    RCLCPP_DEBUG(this->get_logger(), "Menu Item %d clicked (mo)",
                 feedback->menu_entry_id);

    break;

  case InteractiveMarkerFeedback::POSE_UPDATE: {
    RCLCPP_DEBUG(this->get_logger(),
                 "Marker moved to: position (x: %.2f, y: %.2f, z: %.2f), "
                 "orientation (w: %.2f, x: %.2f, y: %.2f, z: %.2f)",
                 feedback->pose.position.x, feedback->pose.position.y,
                 feedback->pose.position.z, feedback->pose.orientation.w,
                 feedback->pose.orientation.x, feedback->pose.orientation.y,
                 feedback->pose.orientation.z);

  }

  break;

  case InteractiveMarkerFeedback::MOUSE_DOWN:
    RCLCPP_DEBUG(this->get_logger(), ": mouse down .");
    break;

  case InteractiveMarkerFeedback::MOUSE_UP:
    RCLCPP_DEBUG(this->get_logger(), ": mouse up .");
    hexapod_msgs::msg::Pose pose;
    pose.names = {leg_name};
    pose.positions = {feedback->pose.position};
    pose_pub_->publish(pose);
    break;
  }
};
