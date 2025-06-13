// TODO: Rename to HexapodPoseControl
// - [ ] Add Context menu
// - [ ] Add Buttons to markers
// - [ ] Try other IK algorithms
// - [ ] Make algorithm more performant and accurate
// - [ ] Show Trail in Rviz for legs
// - [ ] Show reachable regions
// - [ ] Give each control a different color
// - [ ] Dont show control when joint is not active
// - [ ] Fix TF and RobotModel warning
#include "6_dof_marker.cpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "hexapod_control/requests.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/set_marker_array.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include <hexapod_control/hexapod.hpp>
#include <hexapod_control/requests.hpp>
#include <hexapod_control/ros_constants.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <iterator>
#include <map>
#include <rclcpp/client.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

void processFeedback(
    rclcpp::Node::SharedPtr node,
    rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr client,
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
        &feedback) {

  switch (feedback->event_type) {
  case InteractiveMarkerFeedback::BUTTON_CLICK:
    RCLCPP_DEBUG(node->get_logger(), "Button Clicked");
    break;

  case InteractiveMarkerFeedback::MENU_SELECT:
    RCLCPP_DEBUG(node->get_logger(), "Menu Item %d clicked (mo)",
                 feedback->menu_entry_id);

    break;

  case InteractiveMarkerFeedback::POSE_UPDATE: {
    RCLCPP_DEBUG(node->get_logger(),
                 "Marker moved to: position (x: %.2f, y: %.2f, z: %.2f), "
                 "orientation (w: %.2f, x: %.2f, y: %.2f, z: %.2f)",
                 feedback->pose.position.x, feedback->pose.position.y,
                 feedback->pose.position.z, feedback->pose.orientation.w,
                 feedback->pose.orientation.x, feedback->pose.orientation.y,
                 feedback->pose.orientation.z);

  }

  break;

  case InteractiveMarkerFeedback::MOUSE_DOWN:
    RCLCPP_DEBUG(node->get_logger(), ": mouse down .");
    break;

  case InteractiveMarkerFeedback::MOUSE_UP:
    RCLCPP_DEBUG(node->get_logger(), ": mouse up .");
    hexapod_msgs::msg::Pose pose;
    pose.names = {feedback->marker_name};
    pose.positions = {feedback->pose.position};
    setPose(node, client, pose);
    break;
  }
};

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f) {
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

class VisualizationServer : public rclcpp::Node {

public:
  VisualizationServer() : Node("visualization_server") {
    using namespace std::chrono_literals;
    setupROS();
    setupInteractiveMarkers();
  }
  ~VisualizationServer() {}

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr set_pose_client_;
  rclcpp::Service<hexapod_msgs::srv::SetMarkerArray>::SharedPtr
      set_marker_array_service_;

private:
  void poseUpdateCallback(const hexapod_msgs::msg::Pose msg) {
    for (size_t i = 0; i < msg.names.size(); i++) {
      geometry_msgs::msg::Pose pose;
      const std::string &name = msg.names[i];
      pose.position = msg.positions[i];
      server_->setPose(name, pose);
    }
    server_->applyChanges();
  }

  void setupInteractiveMarkers() {
    for (const std::string &leg_name : LEG_NAMES) {
      TranslationMarker marker;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;

      // Adding Menu Items
      marker.menu_handler.insert("Reset");
      // Insert marker and set feedback callback
      marker.name = leg_name;
      marker.header.frame_id = "base_footprint";
      server_->insert(marker);
      server_->setCallback(
          leg_name,
          [this](const visualization_msgs::msg::InteractiveMarkerFeedback::
                     ConstSharedPtr &feedback) {
            processFeedback(shared_from_this(), set_pose_client_, feedback);
          });
    }
    // Apply changes to server
    server_->applyChanges();
  }

  void setupROS() {
    pose_sub_ = this->create_subscription<hexapod_msgs::msg::Pose>(
        POSE_TOPIC,
        10, // QoS history depth
        std::bind(&VisualizationServer::poseUpdateCallback, this,
                  std::placeholders::_1));
    set_pose_client_ =
        this->create_client<hexapod_msgs::srv::SetPose>(SET_POSE_SERVICE_NAME);
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        INTERACTIVE_MARKERS_SERVER_NAME, this, rclcpp::QoS(10),
        rclcpp::QoS(10));

    set_marker_array_service_ = create_service<
        hexapod_msgs::srv::SetMarkerArray>(
        SET_MARKER_ARRAY_SERVICE_NAME,
        [this](const std::shared_ptr<hexapod_msgs::srv::SetMarkerArray::Request>
                   request,
               std::shared_ptr<hexapod_msgs::srv::SetMarkerArray::Response>
                   response) {
          for (size_t i = 0; i < request->leg_names.size(); i++) {
            const std::string &leg_name = request->leg_names[i];
            const std::vector<geometry_msgs::msg::Point> &positions =
                request->trajectories[i].points;
            publishMarkers(leg_name, positions);
          }
          response->success = true;
          response->message = "Successfully Set Marker Array";
        });
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        MARKER_ARRAY_TOPIC, rclcpp::QoS(10));
  }

  void publishMarkers(const std::string leg_name,
                      const std::vector<geometry_msgs::msg::Point> &positions,
                      int action = visualization_msgs::msg::Marker::ADD) {
    visualization_msgs::msg::MarkerArray marker_array;
    std::map<std::string, size_t> leg_indices{
        {"top_left", 1000},  {"mid_left", 2000},  {"bottom_left", 3000},
        {"top_right", 4000}, {"mid_right", 5000}, {"bottom_right", 6000}};
    // FIXME: Store this in YAML Config file
    std::map<std::string, std_msgs::msg::ColorRGBA> leg_colors = {
        {"top_left", makeColor(1.0, 0.0, 0.0)},
        {"mid_left", makeColor(0.0, 1.0, 0.0)},
        {"bottom_left", makeColor(0.0, 0.0, 1.0)},
        {"top_right", makeColor(1.0, 1.0, 0.0)},
        {"mid_right", makeColor(0.0, 1.0, 1.0)},
        {"bottom_right", makeColor(1.0, 0.0, 1.0)}};

    int marker_id = leg_indices[leg_name];

    for (size_t i = 0; i < positions.size(); i++) {
      const auto &position = positions[i];
      // Create a sphere marker for this leg at this pose
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "base_footprint"; // Or your TF frame
      sphere.header.stamp = rclcpp::Clock().now();
      sphere.ns = "leg_spheres";
      sphere.id = marker_id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = action;
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
      if (i > 0) {
        visualization_msgs::msg::Marker arrow;
        arrow.header = sphere.header;
        arrow.ns = "leg_arrows";
        arrow.id = marker_id++;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = action;
        arrow.points.push_back(positions[i - 1]);
        arrow.points.push_back(position);
        arrow.scale.x = 0.0025; // shaft diameter
        arrow.scale.y = 0.01;   // head diameter
        arrow.scale.z = 0.01;   // head length
        arrow.color = sphere.color;
        arrow.pose.orientation.w = 1.0;
        arrow.lifetime = rclcpp::Duration::from_seconds(0);
        marker_array.markers.push_back(arrow);
      }
    }
    markers_pub_->publish(marker_array);
  }

  //
  // void clearMarkers() {
  //   RCLCPP_INFO(get_logger(), "Cleaning Markers");
  //   visualization_msgs::msg::Marker delete_all_marker;
  //   delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  //   visualization_msgs::msg::MarkerArray marker_array;
  //   marker_array.markers = {delete_all_marker};
  //   markers_pub_->publish(marker_array);
  // }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualizationServer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
