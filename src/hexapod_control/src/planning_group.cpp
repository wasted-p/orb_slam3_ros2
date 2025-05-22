#include "hexapod_control/planning_group.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

int PlanningGroup::calculateJntArray(KDL::Chain chain,
                                     geometry_msgs::msg::Point point,
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
PlanningGroup::calculatePosition(KDL::Chain chain,
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
