#include <geometry_msgs/msg/point.hpp>
#include <hexapod_msgs/msg/command.hpp>
#include <hexapod_msgs/msg/gait.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class GaitPlannerNode : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS2 Subscriptions
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr leg_poses_sub_;
  rclcpp::Subscription<hexapod_msgs::msg::Command>::SharedPtr command_sub_;

  // ROS2 Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;
  rclcpp::Publisher<hexapod_msgs::msg::Gait>::SharedPtr hexapod_gait_pub_;

  // ROS Message variables
  hexapod_msgs::msg::Pose pose_msg_;
  hexapod_msgs::msg::Gait gait_;

public:
  GaitPlannerNode() : Node("gait_planner_node") {
    using namespace std::chrono_literals;
    setupROS();
  }

private:
  void timer_callback() { hexapod_gait_pub_->publish(gait_); }

  void command_callback(const hexapod_msgs::msg::Command command) {
    RCLCPP_INFO(get_logger(), "Command %s", command.command_type.c_str());
    if (command.command_type.compare("save") == 0) {
    } else if (command.command_type.compare("change") == 0) {
      RCLCPP_INFO(get_logger(), "Changing Pose to %s",
                  command.pose_name.c_str());
    }
  }

  void visualizeGait(hexapod_msgs::msg::Gait gait) {

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

  void legPoseCallback(const hexapod_msgs::msg::Pose pose) {
    RCLCPP_INFO(get_logger(), "Recieved leg pose %s", pose.name.c_str());
  }

  void setupROS() {
    hexapod_gait_pub_ = this->create_publisher<hexapod_msgs::msg::Gait>(
        "/hexapod/gait", rclcpp::QoS(10));

    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/hexapod/visualization/leg_pose_markers", rclcpp::QoS(10));

    leg_poses_sub_ = this->create_subscription<hexapod_msgs::msg::Pose>(
        "hexapod_control/leg_pose/update", // topic name
        10,                                // QoS history depth
        std::bind(&GaitPlannerNode::legPoseCallback, this,
                  std::placeholders::_1));

    command_sub_ = this->create_subscription<hexapod_msgs::msg::Command>(
        "hexapod_control/command", // topic name
        10,                        // QoS history depth
        std::bind(&GaitPlannerNode::command_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GaitPlannerNode::timer_callback, this));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
