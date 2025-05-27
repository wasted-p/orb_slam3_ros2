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
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include "marker.cpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_control/planning_group.hpp>
#include <hexapod_msgs/msg/command.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <iterator>
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
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class LegControlNode : public rclcpp::Node {
private:
  hexapod_msgs::msg::Pose last_pose_msg_;
  const static int CHAIN_COUNT = 6;
  std::map<std::string, KDL::Chain> chains_;
  sensor_msgs::msg::JointState joint_state_msg_;
  KDL::Tree kdl_tree_;
  std::string urdf_string;
  bool robot_description_loaded_ = false;
  PlanningGroup planning_group;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<hexapod_msgs::msg::Command>::SharedPtr command_sub_;
  hexapod_msgs::msg::Pose pose_msg_;

  std::string LEG_NAMES[6] = {
      "top_left",  "mid_left",  "bottom_left",
      "top_right", "mid_right", "bottom_right",
  };

public:
  LegControlNode() : Node("leg_control_node") {
    using namespace std::chrono_literals;

    readRobotDescription();
    initInteractiveMarkerServer();
    setupROS();

    geometry_msgs::msg::Point rest_pos;
    for (std::string leg_name : LEG_NAMES) {
      KDL::Chain chain;
      kdl_tree_.getChain("base_footprint", leg_name + "_foot", chain);
      chains_.insert({leg_name, chain});

      rest_pos =
          planning_group.calculatePosition(chains_.at(leg_name), {0, 0, 0});

      setupControl(leg_name, rest_pos);

      pose_msg_.names.push_back(leg_name);
      pose_msg_.positions.push_back(rest_pos);

      RCLCPP_DEBUG(get_logger(), "Setting %s to [%.4f,%.4f,%.4f]",
                   leg_name.c_str(), rest_pos.x, rest_pos.y, rest_pos.z);
    };

    pose_pub_->publish(pose_msg_);
  }

private:
  void timer_callback() {
    joint_state_msg_.header.stamp = get_clock()->now();

    joint_state_publisher_->publish(joint_state_msg_);
  }

  void poseUpdateCallback(const hexapod_msgs::msg::Pose pose) {
    RCLCPP_DEBUG(get_logger(), "Recieved Pose:");
    for (size_t i = 0; i < pose.names.size(); i++) {
      RCLCPP_DEBUG(get_logger(), " - [%.4f,%.4f,%.4f]", pose.positions[i].x,
                   pose.positions[i].y, pose.positions[i].z);
    }
    updatePose(pose);
  }

  void command_callback(const hexapod_msgs::msg::Command command) {
    RCLCPP_DEBUG(get_logger(), "Command %s", command.command_type.c_str());
  }

  void updatePose(const hexapod_msgs::msg::Pose pose) {
    std::vector<double> joint_positions;
    std::string leg_name;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Pose int_marker_pose;

    joint_state_msg_.name = {};
    joint_state_msg_.position = {};

    for (size_t i = 0; i < pose.names.size(); i++) {
      {
        leg_name = pose.names[i];
        position = pose.positions[i];
        RCLCPP_DEBUG(
            get_logger(),
            "Recieved Leg Pose Message for leg %s with positions=[%.2f, "
            "%.2f, %.2f]",
            leg_name.c_str(), position.x, position.y, position.z);

        int status = planning_group.calculateJntArray(
            chains_.at(leg_name), position, joint_positions);

        if (status < 0) {
          RCLCPP_ERROR(get_logger(), "Error %i", status);
          return;
        }
        joint_state_msg_.header.frame_id = "base_footprint";

        joint_state_msg_.name.insert(joint_state_msg_.name.cend(),
                                     {leg_name + "_rotate_joint",
                                      leg_name + "_abduct_joint",
                                      leg_name + "_retract_joint"});

        joint_state_msg_.position.insert(
            joint_state_msg_.position.cend(),
            {joint_positions[0], joint_positions[1], joint_positions[2]});

        RCLCPP_DEBUG(
            get_logger(), "Sending Target Joint Positions = [%.2f, %.2f, %.2f]",
            joint_positions[0], joint_positions[1], joint_positions[2]);

        int_marker_pose.position = position;
        server_->setPose(leg_name, int_marker_pose);
        server_->applyChanges();
      }
    }
  }

  void readRobotDescription() {
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
        RCLCPP_INFO(this->get_logger(),
                    "Robot description obtained from robot_state_publisher");
        if (kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
          robot_description_loaded_ = true;
          RCLCPP_INFO(this->get_logger(), "Parsed KDL Tree from URDF");
        }
      }
    }
  }

  void setupControl(std::string leg_name, geometry_msgs::msg::Point pos) {
    RCLCPP_DEBUG(get_logger(),
                 "Setting up %s Interactive Control with joints %d",
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

  std_msgs::msg::ColorRGBA makeColor(float r, float g, float b,
                                     float a = 1.0f) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }
  void setupROS() {
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                             rclcpp::QoS(10));

    std::string POSE_TOPIC = "/hexapod/pose";
    pose_sub_ = this->create_subscription<hexapod_msgs::msg::Pose>(
        POSE_TOPIC,
        10, // QoS history depth
        std::bind(&LegControlNode::poseUpdateCallback, this,
                  std::placeholders::_1));
    pose_pub_ = this->create_publisher<hexapod_msgs::msg::Pose>(
        POSE_TOPIC, rclcpp::QoS(10));

    command_sub_ = this->create_subscription<hexapod_msgs::msg::Command>(
        "hexapod_control/command", // topic name
        10,                        // QoS history depth
        std::bind(&LegControlNode::command_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LegControlNode::timer_callback, this));
  }

  void initInteractiveMarkerServer() {
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_marker_server", this, rclcpp::QoS(10), rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "Interactive marker server started");
  }

  TranslationMarker add6DofControl() {
    TranslationMarker marker;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    // Adding Menu Items
    marker.menu_handler.insert("Reset");
    // Insert marker and set feedback callback
    return marker;
  }

  void processFeedback(
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

      pose_msg_.names = {leg_name};
      pose_msg_.positions = {feedback->pose.position};
      pose_pub_->publish(pose_msg_);
      break;
    }
  };
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LegControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
