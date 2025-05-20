
#ifndef LEG_KINEMATICS_SOLVER_HPP
#define LEG_KINEMATICS_SOLVER_HPP

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <vector>

// Structure to store a single DH parameter set
struct DHParam {
  double theta; // Joint angle offset (θᵢ)
  double d;     // Offset along previous z (dᵢ)
  double a;     // Length of the common normal (aᵢ)
  double alpha; // Angle about common normal, from zᵢ₋₁ to zᵢ (αᵢ)
};

// Kinematic solver class for a 3-DOF robotic leg using DH parameters
class KinematicsSolver {
public:
  // Default constructor
  KinematicsSolver();

  // Set DH parameters (expects exactly 3 for a 3-DOF leg)
  void setDHParams(const std::vector<DHParam> &dhParams);

  // Forward kinematics: returns foot position from given joint angles
  Eigen::Vector3d forwardKinematics(const std::array<double, 3> &jointAngles);

  // Inverse kinematics: returns joint angles to reach given foot position
  std::array<double, 3> inverseKinematics(const Eigen::Vector3d &footPos);

  bool isReachable(const Eigen::Vector3d &footPos) const;

private:
  std::vector<DHParam> dh_; // Stores DH parameters for each link

  // Computes the DH transformation matrix for one joint
  Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha);
};

#endif // LEG_KINEMATICS_SOLVER_HPP
