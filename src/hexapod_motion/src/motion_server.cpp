#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/motion.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/set_marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/set_marker_array.hpp>
#include <memory>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

using namespace std::chrono_literals;
class MotionServer : public rclcpp::Node {

private:
  // ROS2 Subscriptions
  rclcpp::Service<hexapod_msgs::srv::SetMarkerArray>::SharedPtr service_;

  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;

  rclcpp::Subscription<hexapod_msgs::msg::Motion>::SharedPtr action_sub_;

  // ROS Message variables
  // hexapod_msgs::msg::Pose pose_msg_;
  hexapod_msgs::msg::Motion executing_action;
  hexapod_msgs::msg::Pose initial_pose;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, geometry_msgs::msg::Point> buffer;
  std::map<std::string, hexapod_msgs::msg::Motion> actions_;

public:
  MotionServer() : Node("action_server_node") {
    setupROS();
    // FIXME: Pass these params in config file successfully
    // loadMotions();
  }

private:
  void loadMotions() {
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

        hexapod_msgs::msg::Motion gait_msg;
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

          // gait_msg.poses.push_back(pose_msg);
        }

        actions_[gait_id] = gait_msg;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %lu actions.", actions_.size());
  }

  void setupROS() {
    std::string POSE_TOPIC = "/hexapod/pose";

    pose_pub_ =
        create_publisher<hexapod_msgs::msg::Pose>(POSE_TOPIC, rclcpp::QoS(10));

    action_sub_ = this->create_subscription<hexapod_msgs::msg::Motion>(
        "/hexapod/action",
        10, // QoS history depth
        std::bind(&MotionServer::onMotionRequested, this,
                  std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::milliseconds(200),
                               std::bind(&MotionServer::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Created ControlCommand Service");
  }

  void onMotionRequested(hexapod_msgs::msg::Motion msg) {
    RCLCPP_DEBUG(get_logger(), "Requested Motion = %s", msg.name.c_str());
    executing_action = msg;
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
  auto node = std::make_shared<MotionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
