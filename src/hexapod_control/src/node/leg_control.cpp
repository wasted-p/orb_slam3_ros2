// TODO:
// - [ ] Add Context menu
// - [ ] Add Buttons to markers
// - [ ] Try other IK algorithms
// - [ ] Make algorithm more performant and accurate
// - [ ] Show Trail in Rviz for legs
// - [ ] Show reachable regions
// - [ ] Give each control a different color
//
#include "../utils/urdf_parser.cpp"
#include "kinematics/solver.hpp"
#include "marker.cpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
// FIXME: Remove unneeded includes
// NOTE: Use most performant and appropriate algorithms
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

class PlanningGroup {
private:
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> iksolver_;
  unsigned int joint_count;

public:
  PlanningGroup(const KDL::Chain &chain) {
    joint_count = chain.getNrOfJoints();
    fksolver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
    iksolver_ =
        std::make_unique<KDL::ChainIkSolverPos_LMA>(KDL::ChainIkSolverPos_LMA(
            chain, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}, 1e-3, 100, 1e-5));
  };

  int calculateJntArray(Eigen::Vector3d point,
                        std::vector<double> &joint_positions) {
    KDL::Frame T_base_goal;
    T_base_goal.p.y(point.y());
    T_base_goal.p.x(point.x());
    T_base_goal.p.z(point.z());

    KDL::JntArray q_out = KDL::JntArray(3);

    KDL::JntArray q_init = KDL::JntArray(3);
    q_init(0) = 0.4;
    q_init(1) = -0.7;
    q_init(2) = 1.0;

    // Calculate forward position kinematics
    int status = iksolver_->CartToJnt(q_init, T_base_goal, q_out);

    joint_positions = {q_out.data.x(), q_out.data.y(), q_out.data.z()};
    return status;
  }

  Eigen::Vector3d calculatePosition(std::array<double, 3> joint_position_arr) {
    KDL::Frame position;
    KDL::JntArray joint_positions(3);
    for (size_t i = 0; i < 3; i++) {
      joint_positions(i) = joint_position_arr[i];
    }

    int kinematics_status = fksolver_->JntToCart(joint_positions, position);

    // std::cout << position.p.x() << " " << position.p.y() << " "
    //           << position.p.y() << ", " << kinematics_status << std::endl;

    if (kinematics_status < 0)
      throw std::runtime_error("Forward Kinematics Solution not found");

    return Eigen::Vector3d(position.p.x(), position.p.y(),
                           position.p.z()); // Direct array access
  };
};

class LegControlNode : public rclcpp::Node {

private:
  const static int CHAIN_COUNT = 6;
  std::string leg_positions[CHAIN_COUNT] = {
      "top_left",  "mid_left",  "bottom_left",
      "top_right", "mid_right", "bottom_right",
  };
  KDL::Chain chains_[CHAIN_COUNT];

public:
  LegControlNode() : Node("leg_control_node") {
    initialize();
    // Declare URDF string paramater

    RCLCPP_INFO(this->get_logger(), "Interactive marker server started");
  }

private:
  KDL::Tree kdl_tree_;
  std::string urdf_string;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;

  void setupControl(unsigned int idx) {
    PlanningGroup top_left = PlanningGroup(chains_[idx]);
    Eigen::Vector3d rest_pos = top_left.calculatePosition({0, 0, 0});

    TranslationMarker marker = add6DofControl(rest_pos, "base_footprint");
    marker.name = leg_positions[idx];

    server_->insert(marker);
    server_->setCallback(
        marker.name,
        [this, idx](const visualization_msgs::msg::InteractiveMarkerFeedback::
                        ConstSharedPtr &feedback) {
          processFeedback(idx, feedback);
        });

    // Apply changes to server
    server_->applyChanges();
  }
  void initialize() {
    this->declare_parameter<std::string>("robot_description", "");

    if (this->get_parameter("robot_description", urdf_string)) {
      RCLCPP_INFO(this->get_logger(), "Received URDF of length: %ld",
                  urdf_string.length());
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to get 'robot_description' parameter.");
    }

    // Parse URDF string to KDL tree
    if (!kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to construct KDL tree from URDF");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Constructed KDL Tree from URDf");
    }

    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_marker_server", this, rclcpp::QoS(10), rclcpp::QoS(10));

    for (int i = 0; i < CHAIN_COUNT; i++) {
      kdl_tree_.getChain("base_footprint", leg_positions[i] + "_foot",
                         chains_[i]);
      setupControl(i);
    }
  }

  TranslationMarker add6DofControl(Eigen::Vector3d point,
                                   std::string frame_id) {
    TranslationMarker marker;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.header.frame_id = frame_id;

    // Adding Menu Items
    marker.menu_handler.insert("Reset");
    // Insert marker and set feedback callback
    return marker;
  }

  void processFeedback(
      unsigned int idx,
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr
          &feedback) {

    switch (feedback->event_type) {

      // NOTE: Add coord information like in POSE_UPDATE
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      RCLCPP_INFO(this->get_logger(), "Button Clicked");
      break;

    case InteractiveMarkerFeedback::MENU_SELECT:
      RCLCPP_INFO(this->get_logger(), "Menu Item %d clicked (mo)",
                  feedback->menu_entry_id);
      break;

    case InteractiveMarkerFeedback::POSE_UPDATE: {
      RCLCPP_INFO(this->get_logger(),
                  "Marker moved to: position (x: %.2f, y: %.2f, z: %.2f), "
                  "orientation (w: %.2f, x: %.2f, y: %.2f, z: %.2f)",
                  feedback->pose.position.x, feedback->pose.position.y,
                  feedback->pose.position.z, feedback->pose.orientation.w,
                  feedback->pose.orientation.x, feedback->pose.orientation.y,
                  feedback->pose.orientation.z);

      PlanningGroup planning_group = PlanningGroup(chains_[idx]);

      const std::string frame_id = feedback->header.frame_id;
      const auto pos = feedback->pose.position;

      joint_state_publisher_ =
          this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                               rclcpp::QoS(10));

      std::vector<double> joint_positions;
      int status = planning_group.calculateJntArray({pos.x, pos.y, pos.z},
                                                    joint_positions);
      if (status < 0) {
        RCLCPP_INFO(get_logger(), "Error: %i", status);
        break;
      }
      auto joint_state_msg = sensor_msgs::msg::JointState();
      joint_state_msg.header.stamp = now(); // or this->now() in a
      joint_state_msg.name = {
          leg_positions[idx] + "_rotate_joint",
          leg_positions[idx] + "_abduct_joint",
          leg_positions[idx] + "_retract_joint",
      };
      joint_state_msg.header.frame_id = "base_footprint";
      joint_state_msg.position = joint_positions;

      joint_state_publisher_->publish(joint_state_msg);

      RCLCPP_INFO(get_logger(),
                  "Sending Target Joint Positions = [%.2f, %.2f, %.2f]",
                  joint_positions[0], joint_positions[1], joint_positions[2]);

    }

    break;

    case InteractiveMarkerFeedback::MOUSE_DOWN:
      RCLCPP_INFO(this->get_logger(), ": mouse down .");
      break;

    case InteractiveMarkerFeedback::MOUSE_UP:
      RCLCPP_INFO(this->get_logger(), ": mouse up .");
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
