#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/save_pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <exception>
#include <format>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <hexapod_msgs/msg/command.hpp>
#include <hexapod_msgs/msg/gait.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/command.hpp>
#include <hexapod_msgs/srv/save_pose.hpp>
#include <memory>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
class GaitPlannerNode : public rclcpp::Node {

private:
  int pose_idx = 1;
  // ROS2 Subscriptions
  rclcpp::Service<hexapod_msgs::srv::Command>::SharedPtr service_;

  // ROS2 Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;
  // ROS2 Publishers
  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;

  // ROS Message variables
  hexapod_msgs::msg::Pose pose_msg_;
  hexapod_msgs::msg::Gait gait_;

  rclcpp::TimerBase::SharedPtr timer_;

public:
  GaitPlannerNode() : Node("gait_planner_node") { setupROS(); }

private:
  void setupROS() {
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/hexapod/visualization/leg_pose_markers", rclcpp::QoS(10));

    std::string POSE_TOPIC = "/hexapod/pose";
    pose_pub_ =
        create_publisher<hexapod_msgs::msg::Pose>(POSE_TOPIC, rclcpp::QoS(10));

    pose_sub_ = this->create_subscription<hexapod_msgs::msg::Pose>(
        POSE_TOPIC,
        10, // QoS history depth
        std::bind(&GaitPlannerNode::onPoseUpdate, this, std::placeholders::_1));

    timer_ =
        create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&GaitPlannerNode::timerCallback, this));

    service_ = create_service<hexapod_msgs::srv::Command>(
        "command", std::bind(&GaitPlannerNode::handleCommand, this,
                             std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Save gait pose service ready.");
  }

  void onPoseUpdate(hexapod_msgs::msg::Pose pose) { pose_msg_ = pose; }

  void clearMarkers() {
    visualization_msgs::msg::Marker delete_all_marker;
    delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers = {delete_all_marker};
    markers_pub_->publish(marker_array);
  }
  void updateMarkers(hexapod_msgs::msg::Gait gait) {
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
    for (hexapod_msgs::msg::Pose pose : gait.poses) {
      for (size_t leg_i = 0; leg_i < pose.names.size(); ++leg_i) {
        const auto &leg_name = pose.names[leg_i];
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

  void handleCommand(
      const std::shared_ptr<hexapod_msgs::srv::Command::Request> request,
      std::shared_ptr<hexapod_msgs::srv::Command::Response> response) {

    // Do something with request->pose...
    RCLCPP_INFO(get_logger(), "Recieved command %s", request->type.c_str());
    if (request->type.compare("add_pose") == 0) {
      RCLCPP_INFO(get_logger(), "Received Add Pose Request");
      std::string new_pose_name = "Pose " + std::to_string(pose_idx);
      pose_msg_.name = new_pose_name;
      gait_.poses.push_back(pose_msg_);

      response->pose_names = {new_pose_name};
      response->success = true;
      response->message = "Pose added successfully";
      updateMarkers(gait_);
      pose_idx++;
    } else if (request->type.compare("set_pose") == 0) {
      RCLCPP_INFO(get_logger(), "Received Set Pose Request");

      RCLCPP_INFO(get_logger(), "Target Pose Name=%s",
                  request->pose_name.c_str());
      for (size_t i = 0; i < gait_.poses.size(); i++) {
        if (gait_.poses[i].name.compare(request->pose_name.c_str()) == 0) {
          pose_pub_->publish(gait_.poses[i]);
          for (size_t j = 0; j < gait_.poses[i].names.size(); j++) {
            RCLCPP_INFO(
                get_logger(), "Pose Name=%s, [%.4f,%.4f,%.4f]",
                gait_.poses[i].names[j].c_str(), gait_.poses[i].positions[j].x,
                gait_.poses[i].positions[j].y, gait_.poses[i].positions[j].z);
          }
        }
      }
      response->success = true;
      response->message = "Pose set successfully";
    } else if (request->type.compare("delete_pose") == 0) {
      RCLCPP_INFO(get_logger(), "Deleting (%s) from Gait %s",
                  request->pose_name.c_str(), gait_.name.c_str());
      for (size_t i = 0; i < gait_.poses.size(); i++) {
        if (gait_.poses[i].name.compare(request->pose_name.c_str()) == 0) {
          gait_.poses.erase(gait_.poses.cbegin() + i);
        }
      }

      RCLCPP_INFO(get_logger(), "Remaining Poses:");
      for (size_t i = 0; i < gait_.poses.size(); i++) {
        RCLCPP_INFO(get_logger(), " - %s", gait_.poses[i].name.c_str());
      }
      response->success = true;
      response->message = "Pose set successfully";
      clearMarkers();
      updateMarkers(gait_);
      pose_pub_->publish(gait_.poses[gait_.poses.size() - 1]);
    } else if (request->type.compare("save_gait") == 0) {
      try {
        saveGaitToYamlFile(request->filepath);
        response->success = true;
        response->message = "Gait saved successfully";
        RCLCPP_INFO(this->get_logger(), "Saved gait to %s",
                    request->filepath.c_str());
      } catch (const std::exception &e) {

        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s",
                     request->filepath.c_str());
      }
    } else if (request->type.compare("load_gait") == 0) {
      try {
        loadGaitFromYamlFile(request->filepath, gait_);
        clearMarkers();
        updateMarkers(gait_);
        pose_pub_->publish(gait_.poses[0]);

        for (hexapod_msgs::msg::Pose pose : gait_.poses) {
          response->pose_names.push_back(pose.name);
        }
        response->success = true;
        response->message = "Gait loaded successfully";
        RCLCPP_INFO(this->get_logger(), "Loaded gait to %s",
                    request->filepath.c_str());
      } catch (const std::exception &e) {
        response->success = false;
        response->message = "Error occurred";
        RCLCPP_ERROR(this->get_logger(), "Error Loading Gait: %s",
                     request->filepath.c_str());
      }
    }
  }

  void saveGaitToYamlFile(std::string path) {
    std::ofstream file(path);
    if (!file.is_open())
      throw std::runtime_error("Failed to open file for writing");

    file << "gait:\n";
    file << "  name: " << gait_.name << "\n";
    file << "  poses:\n";

    for (const auto &pose : gait_.poses) {
      file << "    - name: " << pose.name << "\n";
      file << "      names:\n";
      for (const auto &leg_name : pose.names) {
        file << "        - " << leg_name << "\n";
      }
      file << "      positions:\n";
      for (const auto &pos : pose.positions) {
        file << "        - {x: " << pos.x << ", y: " << pos.y
             << ", z: " << pos.z << "}\n";
      }
    }
    file.close();
  }

  void loadGaitFromYamlFile(const std::string &filepath,
                            hexapod_msgs::msg::Gait &gait_msg) {
    YAML::Node root = YAML::LoadFile(filepath);
    auto gait_node = root["gait"];
    if (!gait_node)
      throw std::runtime_error("No 'gait' key in YAML.");

    gait_msg.name = gait_node["name"].as<std::string>();
    gait_msg.poses = {};

    auto poses_node = gait_node["poses"];
    if (!poses_node || !poses_node.IsSequence()) {
      throw std::runtime_error("'poses' is missing or not a sequence");
    }

    for (const auto &pose_node : poses_node) {
      hexapod_msgs::msg::Pose pose_msg;

      pose_msg.name = pose_node["name"].as<std::string>();

      // Load names
      auto names_node = pose_node["names"];
      if (names_node && names_node.IsSequence()) {
        for (const auto &name_entry : names_node) {
          pose_msg.names.push_back(name_entry.as<std::string>());
        }
      }

      // Load positions
      auto pos_node = pose_node["positions"];
      if (pos_node && pos_node.IsSequence()) {
        for (const auto &p : pos_node) {
          geometry_msgs::msg::Point pt;
          pt.x = p["x"].as<double>();
          pt.y = p["y"].as<double>();
          pt.z = p["z"].as<double>();
          pose_msg.positions.push_back(pt);
        }
      }

      gait_msg.poses.push_back(pose_msg);
    }
  }

  void timerCallback() {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
