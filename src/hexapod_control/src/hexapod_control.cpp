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
#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/control_command.hpp"
#include "hexapod_msgs/msg/leg_pose.hpp"
#include "hexapod_msgs/msg/leg_poses.hpp"
#include "marker.cpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_control/planning_group.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// FIXME: Remove unneeded includes
// NOTE: Use most performant and appropriate algorithms
// #include "hexapod_control/msg/show_marker.hpp"
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
#include <urdf/model.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

class LegControlNode : public rclcpp::Node {

private:
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
  rclcpp::Publisher<hexapod_msgs::msg::LegPoses>::SharedPtr leg_poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;
  rclcpp::Subscription<hexapod_msgs::msg::LegPose>::SharedPtr leg_pose_sub_;
  rclcpp::Subscription<hexapod_msgs::msg::ControlCommand>::SharedPtr
      command_sub_;
  hexapod_msgs::msg::LegPoses leg_poses_msg_;
  std::vector<hexapod_msgs::msg::LegPoses> saved_leg_poses_;

public:
  LegControlNode() : Node("leg_control_node") {
    using namespace std::chrono_literals;

    std::string LEG_NAMES[6] = {
        "top_left",  "mid_left",  "bottom_left",
        "top_right", "mid_right", "bottom_right",
    };

    readRobotDescription();
    initInteractiveMarkerServer();
    setupROS();

    joint_state_msg_.header.frame_id = "base_footprint";
    joint_state_msg_.header.stamp = get_clock()->now(); // or this->now() in a
    joint_state_msg_.header.frame_id = "base_footprint";

    for (std::string leg_name : LEG_NAMES) {
      KDL::Chain chain;
      kdl_tree_.getChain("base_footprint", leg_name + "_foot", chain);
      chains_.insert({leg_name, chain});

      geometry_msgs::msg::Point rest_pos =
          planning_group.calculatePosition(chains_.at(leg_name), {0, 0, 0});
      joint_state_msg_.header.stamp = get_clock()->now(); // or this->now()
      joint_state_msg_.name.insert(joint_state_msg_.name.cend(),
                                   {

                                       leg_name + "_rotate_joint",
                                       leg_name + "_abduct_joint",
                                       leg_name + "_retract_joint",
                                   });
      joint_state_msg_.position.insert(joint_state_msg_.position.cend(),
                                       {

                                           0.0, 0.0, 0.0});

      setupControl(leg_name, rest_pos);

      leg_poses_msg_.leg_names.push_back(leg_name);
      leg_poses_msg_.positions.push_back(rest_pos);
      RCLCPP_DEBUG(get_logger(), "Setting %s to [%.4f,%.4f,%.4f]",
                   leg_name.c_str(), rest_pos.x, rest_pos.y, rest_pos.z);
    };

    saved_leg_poses_.push_back(leg_poses_msg_);
  }

private:
  void timer_callback() {
    joint_state_msg_.header.stamp = get_clock()->now();

    joint_state_publisher_->publish(joint_state_msg_);

    leg_poses_pub_->publish(leg_poses_msg_);
  }

  void leg_pose_callback(const hexapod_msgs::msg::LegPose pose) {
    updateLegPose(pose);
  }

  void command_callback(const hexapod_msgs::msg::ControlCommand command) {
    RCLCPP_INFO(get_logger(), "Command %s", command.command_type.c_str());
    if (command.command_type.compare("save") == 0) {
      RCLCPP_INFO(get_logger(), "Saving Pose");
      savePose();
    }
  }

  void updateLegPose(const hexapod_msgs::msg::LegPose leg_pose) {
    std::vector<double> joint_positions;

    RCLCPP_DEBUG(get_logger(),
                 "Recieved Leg Pose Message for leg %s with positions=[%.2f, "
                 "%.2f, %.2f]",
                 leg_pose.leg_name.c_str(), leg_pose.position.x,
                 leg_pose.position.y, leg_pose.position.z);

    int status = planning_group.calculateJntArray(
        chains_.at(leg_pose.leg_name), leg_pose.position, joint_positions);

    if (status < 0) {
      RCLCPP_ERROR(get_logger(), "Error %i", status);
      return;
    }

    joint_state_msg_.header.stamp = get_clock()->now(); // or this->now() in a
    joint_state_msg_.name = {
        leg_pose.leg_name + "_rotate_joint",
        leg_pose.leg_name + "_abduct_joint",
        leg_pose.leg_name + "_retract_joint",
    };
    joint_state_msg_.header.frame_id = "base_footprint";
    joint_state_msg_.position = joint_positions;

    RCLCPP_DEBUG(get_logger(),
                 "Sending Target Joint Positions = [%.2f, %.2f, %.2f]",
                 joint_positions[0], joint_positions[1], joint_positions[2]);

    geometry_msgs::msg::Pose pose;
    pose.position = leg_pose.position;
    server_->setPose(leg_pose.leg_name, pose);
    server_->applyChanges();

    auto it = find(leg_poses_msg_.leg_names.begin(),
                   leg_poses_msg_.leg_names.end(), leg_pose.leg_name);
    if (it != leg_poses_msg_.leg_names.end()) {
      size_t index = std::distance(leg_poses_msg_.leg_names.begin(), it);
      leg_poses_msg_.positions[index] = leg_pose.position;
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

  void savePose() {
    visualization_msgs::msg::MarkerArray marker_array;

    saved_leg_poses_.push_back(leg_poses_msg_);
    RCLCPP_INFO(get_logger(), "Saved pose");
    std::map<std::string, std_msgs::msg::ColorRGBA> leg_colors = {
        {"top_left", makeColor(1.0, 0.0, 0.0)},
        {"mid_left", makeColor(0.0, 1.0, 0.0)},
        {"bottom_left", makeColor(0.0, 0.0, 1.0)},
        {"top_right", makeColor(1.0, 1.0, 0.0)},
        {"mid_right", makeColor(0.0, 1.0, 1.0)},
        {"bottom_right", makeColor(1.0, 0.0, 1.0)}};

    // Store previous pose per leg to draw arrows
    std::map<std::string, geometry_msgs::msg::Point> previous_points;

    int marker_id = 0;

    for (size_t pose_index = 0; pose_index < saved_leg_poses_.size();
         ++pose_index) {
      const auto &pose = saved_leg_poses_[pose_index];

      for (size_t leg_i = 0; leg_i < pose.leg_names.size(); ++leg_i) {
        const auto &leg_name = pose.leg_names[leg_i];
        const auto &position = pose.positions[leg_i];

        // Create a sphere marker for this leg at this pose
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "base_footprint"; // Or your TF frame
        sphere.header.stamp = rclcpp::Clock().now();
        sphere.ns = "leg_spheres";
        sphere.id = marker_id++;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position = position;
        sphere.pose.orientation.w = 1.0;
        const double size = 0.0075;
        sphere.scale.x = size;
        sphere.scale.y = size;
        sphere.scale.z = size;
        sphere.color = leg_colors[leg_name];
        sphere.lifetime = rclcpp::Duration::from_seconds(0); // forever

        marker_array.markers.push_back(sphere);

        // If this isn't the first pose, draw an arrow from previous to current
        if (pose_index > 0 && previous_points.count(leg_name)) {
          visualization_msgs::msg::Marker arrow;
          arrow.header = sphere.header;
          arrow.ns = "leg_arrows";
          arrow.id = marker_id++;
          arrow.type = visualization_msgs::msg::Marker::ARROW;
          arrow.action = visualization_msgs::msg::Marker::ADD;
          arrow.points.push_back(previous_points[leg_name]);
          arrow.points.push_back(position);
          arrow.scale.x = 0.0025; // shaft diameter
          arrow.scale.y = 0.01;   // head diameter
          arrow.scale.z = 0.01;   // head length
          arrow.color = sphere.color;
          arrow.pose.orientation.w = 1.0;
          arrow.lifetime = rclcpp::Duration::from_seconds(0);
          marker_array.markers.push_back(arrow);
        }

        previous_points[leg_name] = position;
      }
    }

    markers_pub_->publish(marker_array);
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

    leg_poses_pub_ = this->create_publisher<hexapod_msgs::msg::LegPoses>(
        "hexapod_control/leg_pose", rclcpp::QoS(10));

    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/hexapod/visualization/leg_pose_markers", rclcpp::QoS(10));

    leg_pose_sub_ = this->create_subscription<hexapod_msgs::msg::LegPose>(
        "hexapod_control/leg_pose/update", // topic name
        10,                                // QoS history depth
        std::bind(&LegControlNode::leg_pose_callback, this,
                  std::placeholders::_1));

    command_sub_ = this->create_subscription<hexapod_msgs::msg::ControlCommand>(
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

      // NOTE: Add coord information like in POSE_UPDATE
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

      hexapod_msgs::msg::LegPose pose;
      pose.leg_name = leg_name;
      pose.position = feedback->pose.position;
      updateLegPose(pose);
    }

    break;

    case InteractiveMarkerFeedback::MOUSE_DOWN:
      RCLCPP_DEBUG(this->get_logger(), ": mouse down .");
      break;

    case InteractiveMarkerFeedback::MOUSE_UP:
      RCLCPP_DEBUG(this->get_logger(), ": mouse up .");
      // savePose();
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
