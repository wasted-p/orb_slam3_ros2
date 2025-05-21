
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

class PlanningGroup {
public:
  int calculateJntArray(KDL::Chain chain, Eigen::Vector3d point,
                        std::vector<double> &joint_positions);

  Eigen::Vector3d calculatePosition(KDL::Chain chain,
                                    std::array<double, 3> joint_position_arr);
};
