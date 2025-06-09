#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "hexapod_msgs/msg/action.hpp"
#include "hexapod_msgs/msg/gait.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/control_markers.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <hexapod_msgs/msg/gait.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/control_markers.hpp>
#include <memory>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
class ActionNode : public rclcpp::Node {

private:
  // ROS2 Subscriptions
  rclcpp::Service<hexapod_msgs::srv::ControlMarkers>::SharedPtr service_;

  // ROS2 Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;

  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;

  rclcpp::Subscription<hexapod_msgs::msg::Action>::SharedPtr action_sub_;

  // ROS Message variables
  // hexapod_msgs::msg::Pose pose_msg_;
  hexapod_msgs::msg::Gait tripod_gait_;
  size_t gait_index_ = 100;
  hexapod_msgs::msg::Action executing_action;

  hexapod_msgs::msg::Pose initial_pose;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, geometry_msgs::msg::Point> buffer;
  std::vector<std::string> LEG_NAMES = {
      "top_left",  "mid_left",  "bottom_left",
      "top_right", "mid_right", "bottom_right",
  };
  std::map<std::string, hexapod_msgs::msg::Gait> actions_;

public:
  ActionNode() : Node("action_server_node") {
    setupROS();
    loadActions();
  }

private:
  void loadActions() {
    std::string yaml_file;
    hexapod_msgs::msg::Pose pose_msg;
    this->get_parameter("actions_file", yaml_file);

    if (yaml_file.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No 'actions_file' parameter specified.");
      return;
    }

    YAML::Node root = YAML::LoadFile(yaml_file);
    if (!root["actions"]) {
      RCLCPP_ERROR(this->get_logger(),
                   "YAML file does not contain 'actions' field.");
      return;
    }

    for (const auto &item : root["actions"]) {
      for (const auto &gait_pair : item) {
        std::string gait_id = gait_pair.first.as<std::string>();
        const YAML::Node &gait_node = gait_pair.second;

        hexapod_msgs::msg::Gait gait_msg;
        gait_msg.name = gait_node["name"].as<std::string>();

        for (const auto &pose_node : gait_node["poses"]) {
          pose_msg.name = pose_node["name"].as<std::string>();

          for (const auto &n : pose_node["names"]) {
            pose_msg.names.push_back(n.as<std::string>());
          }

          for (const auto &pos : pose_node["positions"]) {
            geometry_msgs::msg::Point p;
            p.x = pos["x"].as<double>();
            p.y = pos["y"].as<double>();
            p.z = pos["z"].as<double>();
            pose_msg.positions.push_back(p);
          }

          gait_msg.poses.push_back(pose_msg);
        }

        actions_[gait_id] = gait_msg;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %lu actions.", actions_.size());
  }

  hexapod_msgs::msg::Pose getInitialPose() {
    hexapod_msgs::msg::Pose initial_pose;
    // Get the list of leg names
    std::vector<std::string> leg_names_ =
        declare_parameter<std::vector<std::string>>("names",
                                                    std::vector<std::string>());

    if (leg_names_.empty()) {
      throw std::runtime_error("No leg names found in paramaters");
    }

    // Load position for each leg
    for (const auto &leg_name : leg_names_) {
      geometry_msgs::msg::Point position;

      // Read x, y, z coordinates for each leg
      std::string x_param = "positions." + leg_name + ".x";
      std::string y_param = "positions." + leg_name + ".y";
      std::string z_param = "positions." + leg_name + ".z";

      position.x = declare_parameter<double>(x_param, 0.0);
      position.y = declare_parameter<double>(y_param, 0.0);
      position.z = declare_parameter<double>(z_param, 0.0);

      // Store the position
      initial_pose.names.push_back(leg_name);
      initial_pose.positions.push_back(position);

      RCLCPP_INFO(get_logger(), "Loaded position for %s: [%.4f, %.4f, %.4f]",
                  leg_name.c_str(), position.x, position.y, position.z);
    }

    RCLCPP_INFO(get_logger(), "Successfully loaded %zu leg positions",
                leg_names_.size());
    return initial_pose;
  }

  void setupROS() {
    std::string POSE_TOPIC = "/hexapod/pose";

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/hexapod/visualization/leg_pose_markers", rclcpp::QoS(10));

    pose_pub_ =
        create_publisher<hexapod_msgs::msg::Pose>(POSE_TOPIC, rclcpp::QoS(10));

    action_sub_ = this->create_subscription<hexapod_msgs::msg::Action>(
        "/hexapod/action",
        10, // QoS history depth
        std::bind(&ActionNode::onActionRequested, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(200),
                               std::bind(&ActionNode::timerCallback, this));

    service_ = create_service<hexapod_msgs::srv::ControlMarkers>(
        "command", std::bind(&ActionNode::handleControlMarkersRequest, this,
                             std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Created ControlCommand Service");
  }

  void onActionRequested(hexapod_msgs::msg::Action msg) {
    RCLCPP_DEBUG(get_logger(), "Requested Action = %s", msg.name.c_str());
    executing_action = msg;
  }
  void handleControlMarkersRequest(
      const std::shared_ptr<hexapod_msgs::srv::ControlMarkers::Request> request,
      std::shared_ptr<hexapod_msgs::srv::ControlMarkers::Response> response) {
    if (request->command.compare("clear") == 0) {
      clearMarkers();
    } else if (request->command.compare("add") == 0) {
      addMarkers(request->poses);
    }
  }

  void clearMarkers() {

    RCLCPP_INFO(get_logger(), "Clearning Markers");
    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers = {delete_all_marker};
    markers_pub_->publish(marker_array);
  }
  void addMarkers(std::vector<hexapod_msgs::msg::Pose> poses) {
    visualization_msgs::msg::MarkerArray marker_array;
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

    unsigned int idx = 0;

    RCLCPP_INFO(get_logger(), "Adding markers:");
    for (hexapod_msgs::msg::Pose pose : poses) {
      RCLCPP_INFO(get_logger(), " - Pose: %s", pose.name.c_str());
      for (size_t leg_i = 0; leg_i < pose.names.size(); ++leg_i) {

        const auto &leg_name = pose.names[leg_i];
        const auto &position = pose.positions[leg_i];

        RCLCPP_INFO(get_logger(), "   - %s = [%.4f,%.4f,%.4f]",
                    pose.name.c_str(), position.x, position.y, position.z);
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

        // If this isn't the first pose, draw an arrow from previous to
        // current
        if (idx > 0 && previous_points.count(leg_name)) {
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
      idx++;
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

  void timerCallback() {
    hexapod_msgs::msg::Pose pose;
    //   if (executing_action.name.compare("walk") == 0) {
    //     if (gait_index_ >= tripod_gait_.poses.size()) {
    //       gait_index_ = 0;
    //     }
    //     pose = tripod_gait_.poses[gait_index_];
    //     pose.duration = rclcpp::Duration::from_seconds(0.001);
    //     gait_index_++;
    //
    //     pose_pub_->publish(pose);
    //   } else if (executing_action.name.compare("rest") == 0) {
    //
    //     pose_pub_->publish(initial_pose);
    //   }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
