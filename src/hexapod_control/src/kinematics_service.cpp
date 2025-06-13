
#include "geometry_msgs/msg/point.hpp"
#include "hexapod_control/ros_constants.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/solve_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <hexapod_control/hexapod.hpp>
#include <hexapod_msgs/srv/solve_fk.hpp>
#include <hexapod_msgs/srv/solve_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/create_service.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/service.hpp>
#include <vector>
class PlanningGroup {
public:
  int calculateJntArray(KDL::Chain chain, geometry_msgs::msg::Point point,
                        std::vector<double> &joint_positions) {

    unsigned int joint_count = chain.getNrOfJoints();
    KDL::ChainIkSolverPos_LMA iksolver_ = KDL::ChainIkSolverPos_LMA(
        chain, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}, 1e-3, 100, 1e-5);
    KDL::Frame T_base_goal;
    T_base_goal.p.y(point.y);
    T_base_goal.p.x(point.x);
    T_base_goal.p.z(point.z);

    KDL::JntArray q_out = KDL::JntArray(joint_count);

    KDL::JntArray q_init = KDL::JntArray(joint_count);
    q_init(0) = 0.4;
    q_init(1) = -0.7;
    q_init(2) = 1.0;

    // Calculate forward position kinematics
    int status = iksolver_.CartToJnt(q_init, T_base_goal, q_out);

    joint_positions = {q_out.data.x(), q_out.data.y(), q_out.data.z()};
    return status;
  }

  geometry_msgs::msg::Point
  calculatePosition(KDL::Chain chain,
                    std::array<double, 3> joint_position_arr) {
    geometry_msgs::msg::Point result;
    unsigned int joint_count = chain.getNrOfJoints();
    KDL::ChainFkSolverPos_recursive fksolver_ =
        KDL::ChainFkSolverPos_recursive(chain);
    KDL::Frame position;
    KDL::JntArray joint_positions(joint_count);
    for (size_t i = 0; i < 3; i++) {
      joint_positions(i) = joint_position_arr[i];
    }

    int kinematics_status = fksolver_.JntToCart(joint_positions, position);

    if (kinematics_status < 0)
      throw std::runtime_error("Forward Kinematics Solution not found");

    result.x = position.p.x();
    result.y = position.p.y();
    result.z = position.p.z();

    return result;
  };
};

class KinematicsService : public rclcpp::Node {
private:
  const static int CHAIN_COUNT = 6;
  std::map<std::string, KDL::Chain> chains_;
  KDL::Tree kdl_tree_;
  std::string urdf_string;
  bool robot_description_loaded_ = false;
  PlanningGroup planning_group;

public:
  KinematicsService() : Node("kinematics_service") {
    using namespace std::chrono_literals;
    readRobotDescription();
    setupROS();
    setupKDL();
  }
  ~KinematicsService() {}

  void setupKDL() {
    hexapod_msgs::msg::Pose pose;
    geometry_msgs::msg::Point pos;
    for (std::string leg_name : LEG_NAMES) {
      KDL::Chain chain;
      kdl_tree_.getChain("base_footprint", leg_name + "_foot", chain);
      chains_.insert({leg_name, chain});
    };
  }

  void handleSolveFKRequest(
      const std::shared_ptr<hexapod_msgs::srv::SolveFK::Request> request,
      std::shared_ptr<hexapod_msgs::srv::SolveFK::Response> response) {
    std::string leg_name;

    for (size_t i = 0; i < request->leg_names.size(); i++) {
      leg_name = request->leg_names[i];
      // joint_position = request->joint_positions[i];

      // int status = planning_group.calculatePosition(
      //     chains_.at(leg_name), request->position, response->position);
      //
      // if (status < 0) {
      //   RCLCPP_ERROR(
      //       get_logger(),
      //       "IK failed or returned insufficient joint positions for leg "
      //       "%s (status: %d)",
      //       leg_name.c_str(), status);
      response->success = false;
      response->message = "Error Occurred";
      // }
    }
  }

  void handleSolveIKRequest(
      const std::shared_ptr<hexapod_msgs::srv::SolveIK::Request> request,
      std::shared_ptr<hexapod_msgs::srv::SolveIK::Response> response) {
    for (size_t i = 0; i < request->leg_names.size(); i++) {
      std::string &leg_name = request->leg_names[i];
      geometry_msgs::msg::Point &pos = request->positions[i];
      std::vector<double> joint_positions;

      int status = planning_group.calculateJntArray(chains_.at(leg_name), pos,
                                                    joint_positions);

      if (status < 0) {
        RCLCPP_ERROR(
            get_logger(),
            "IK failed or returned insufficient joint positions for leg "
            "%s (status: %d)",
            leg_name.c_str(), status);

        response->success = false;
        response->message = "Error Occurred";
      } else {
        response->success = true;
        response->joint_names.insert(response->joint_names.cbegin(),
                                     {
                                         leg_name + "_rotate_joint",
                                         leg_name + "_abduct_joint",
                                         leg_name + "_retract_joint",
                                     });
        for (const double &x : joint_positions)
          response->joint_positions.push_back(x);
      }
    }
  }

#define ROBOT_DESCRIPTION "robot_description"
  void readRobotDescription() {
    // Declare and get the YAML string parameter
    std::string urdf_string =
        this->declare_parameter(ROBOT_DESCRIPTION, std::string(""));
    RCLCPP_INFO(this->get_logger(),
                "Reading robot_description from paramater (%s)",
                ROBOT_DESCRIPTION);
    parseURDF(urdf_string);
  }

  void parseURDF(const std::string &urdf_string) {
    if (kdl_parser::treeFromString(urdf_string, kdl_tree_)) {
      robot_description_loaded_ = true;
      RCLCPP_INFO(this->get_logger(), "Parsed KDL Tree from urdf_string");
    }
  }

  void readRobotDescriptionFromTopic() {
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
        RCLCPP_DEBUG(this->get_logger(),
                     "Robot description obtained from robot_state_publisher");
      }
    }
  }

  rclcpp::Service<hexapod_msgs::srv::SolveIK>::SharedPtr solve_ik_service_;
  rclcpp::Service<hexapod_msgs::srv::SolveFK>::SharedPtr solve_fk_service_;
  void setupROS() {
    solve_ik_service_ = create_service<hexapod_msgs::srv::SolveIK>(
        SOLVE_IK_SERVICE_NAME,
        std::bind(&KinematicsService::handleSolveIKRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    solve_fk_service_ = create_service<hexapod_msgs::srv::SolveFK>(
        SOLVE_FK_SERVICE_NAME,
        std::bind(&KinematicsService::handleSolveFKRequest, this,
                  std::placeholders::_1, std::placeholders::_2));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KinematicsService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
