#ifndef PLANNING_GROUP_HPP
#define PLANNING_GROUP_HPP

#include <geometry_msgs/msg/point.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

class PlanningGroup {
public:
  int calculateJntArray(KDL::Chain chain, geometry_msgs::msg::Point point,
                        std::vector<double> &joint_positions);

  geometry_msgs::msg::Point
  calculatePosition(KDL::Chain chain, std::array<double, 3> joint_position_arr);
};

#endif
