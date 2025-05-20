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

// FIXME: Remove unneeded includes
// NOTE: Use most performant and appropriate algorithms
// #include "hexapod_control/msg/show_marker.hpp"

#include "hexapod_msgs/msg/leg_pose.hpp"
#include "marker.cpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_msgs/msg/show_marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
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
    readRobotDescription();
    initialize();
  }

private:
  KDL::Tree kdl_tree_;
  std::string urdf_string;
  bool robot_description_loaded_ = false;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;

  rclcpp::Subscription<hexapod_msgs::msg::LegPose>::SharedPtr sub_leg_pose_;

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

    setJointPositions(
        {
            "top_left_rotate_joint",
            "top_left_abduct_joint",
            "top_left_retract_joint",
        },
        {0, 0, 0});
  }

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
  void updateLegPose(const hexapod_msgs::msg::LegPose::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Updating Leg Pose = %s", msg->leg_name.c_str());
    Eigen::Vector3d position = {msg->position.x, msg->position.y,
                                msg->position.z};
    TranslationMarker marker = add6DofControl(position, "base_footprint");
    marker.name = msg->leg_name;
  };
  void initialize() {
    joint_state_publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                             rclcpp::QoS(10));
    // sub_leg_pose_ = create_subscription<hexapod_msgs::msg::LegPose>(
    //     "/hexapod_control/leg_pose/update", 10,
    //     std::bind(&LegControlNode::updateLegPose, this,
    //     std::placeholders::_1));

    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_marker_server", this, rclcpp::QoS(10), rclcpp::QoS(10));

    for (int i = 0; i < CHAIN_COUNT; i++) {
      kdl_tree_.getChain("base_footprint", leg_positions[i] + "_foot",
                         chains_[i]);
      setupControl(i);
    }

    RCLCPP_INFO(this->get_logger(), "Interactive marker server started");
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

  void setJointPositions(std::vector<std::string> names,
                         std::vector<double> positions) {

    sensor_msgs::msg::JointState joint_state_msg =
        sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = get_clock()->now(); // or this->now() in a
    joint_state_msg.name = names;
    joint_state_msg.header.frame_id = "base_footprint";
    joint_state_msg.position = positions;

    joint_state_publisher_->publish(joint_state_msg);
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
      RCLCPP_DEBUG(this->get_logger(),
                   "Marker moved to: position (x: %.2f, y: %.2f, z: %.2f), "
                   "orientation (w: %.2f, x: %.2f, y: %.2f, z: %.2f)",
                   feedback->pose.position.x, feedback->pose.position.y,
                   feedback->pose.position.z, feedback->pose.orientation.w,
                   feedback->pose.orientation.x, feedback->pose.orientation.y,
                   feedback->pose.orientation.z);

      PlanningGroup planning_group = PlanningGroup(chains_[idx]);

      const std::string frame_id = feedback->header.frame_id;
      const auto pos = feedback->pose.position;

      std::vector<double> joint_positions;
      int status = planning_group.calculateJntArray({pos.x, pos.y, pos.z},
                                                    joint_positions);
      if (status < 0) {
        RCLCPP_INFO(get_logger(), "Error: %i", status);
        break;
      }
      setJointPositions(
          {
              leg_positions[idx] + "_rotate_joint",
              leg_positions[idx] + "_abduct_joint",
              leg_positions[idx] + "_retract_joint",
          },
          joint_positions);

      RCLCPP_DEBUG(get_logger(),
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
