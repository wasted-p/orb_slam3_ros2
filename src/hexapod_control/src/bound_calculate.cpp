
#include "marker.cpp"
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <hexapod_control/planning_group.hpp>
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
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

class HexapodModelNode : public rclcpp::Node {

private:
  const static int CHAIN_COUNT = 6;
  KDL::Tree kdl_tree_;
  std::string urdf_string;
  bool robot_description_loaded_ = false;

  std::string LEG_NAMES[6] = {
      "top_left",  "mid_left",  "bottom_left",
      "top_right", "mid_right", "bottom_right",
  };

public:
  HexapodModelNode() : Node("hexapod_model_node") {
    using namespace std::chrono_literals;

    readRobotDescription();

    for (std::string leg_name : LEG_NAMES) {
      KDL::Chain chain;
      kdl_tree_.getChain("base_footprint", leg_name + "_foot", chain);
    };
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
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HexapodModelNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
