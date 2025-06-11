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
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/set_pose.hpp"
#include <hexapod_control/hexapod.hpp>
#include <hexapod_control/ros_constants.hpp>
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/srv/get_pose.hpp>
#include <hexapod_msgs/srv/set_pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <string>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

void processFeedback(
    rclcpp::Node::SharedPtr node,
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
    // pose_pub_->publish(pose);
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
  }
  ~VisualizationServer() {}

private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<hexapod_msgs::srv::SetPose>::SharedPtr set_pose_client_;

private:
  void poseUpdateCallback(const hexapod_msgs::msg::Pose pose) {
    RCLCPP_DEBUG(get_logger(), "Recieved Pose:%s", pose.name.c_str());
  }

  void setupControl(std::string leg_name, geometry_msgs::msg::Point pos) {
    TranslationMarker marker = add6DofControl();
    marker.pose.position = pos;
    marker.name = leg_name;
    marker.header.frame_id = "base_footprint";
    server_->insert(marker);
    server_->setCallback(
        marker.name,
        [this](const visualization_msgs::msg::InteractiveMarkerFeedback::
                   ConstSharedPtr &feedback) {
          processFeedback(shared_from_this(), feedback);
        });

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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualizationServer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
