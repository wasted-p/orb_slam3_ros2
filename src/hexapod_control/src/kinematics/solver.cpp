#include "kinematics/solver.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

KinematicsSolver::KinematicsSolver() {}

void KinematicsSolver::setDHParams(const std::vector<DHParam> &dhParams) {
  if (dhParams.size() != 3)
    throw std::runtime_error("Only 3 DOF supported");
  dh_ = dhParams;
}

Eigen::Vector3d
KinematicsSolver::forwardKinematics(const std::array<double, 3> &jointAngles) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  for (size_t i = 0; i < 3; ++i) {
    double theta = jointAngles[i] + dh_[i].theta; // add joint offset
    T = T * dhTransform(theta, dh_[i].d, dh_[i].a, dh_[i].alpha);
  }

  Eigen::Vector4d foot =
      T * Eigen::Vector4d(0, 0, 0, 1); // homogeneous coordinates
  return foot.head<3>();               // return x, y, z
}

std::array<double, 3>
KinematicsSolver::inverseKinematics(const Eigen::Vector3d &footPos) {
  if (dh_.size() != 3)
    throw std::runtime_error("Expected 3 DH parameters");

  double x = footPos[0];
  double y = footPos[1];
  double z = footPos[2];

  // Effective 3D link lengths using sqrt(a^2 + d^2)
  double l1 = std::sqrt(std::pow(dh_[0].a, 2) + std::pow(dh_[0].d, 2));
  double l2 = std::sqrt(std::pow(dh_[1].a, 2) + std::pow(dh_[1].d, 2));
  double l3 = std::sqrt(std::pow(dh_[2].a, 2) + std::pow(dh_[2].d, 2));

  double theta1 = std::atan2(y, x); // Base rotation in XY plane

  // Project foot position into leg sagittal plane (XZ)
  double x_proj = std::sqrt(x * x + y * y) - l1;
  double d = std::sqrt(x_proj * x_proj + z * z);

  // Reachability check
  double cos_theta3 = (l2 * l2 + l3 * l3 - d * d) / (2 * l2 * l3);
  if (cos_theta3 < -1.0 || cos_theta3 > 1.0)
    throw std::runtime_error("Target point unreachable");

  double theta3 = M_PI - std::acos(cos_theta3); // Elbow angle (knee)

  // Triangle geometry
  double alpha = std::atan2(z, x_proj); // angle to foot from shoulder
  double cos_beta = (l2 * l2 + d * d - l3 * l3) / (2 * l2 * d);
  double beta = std::acos(cos_beta);
  double theta2 = alpha + beta; // Shoulder angle

  // Subtract static DH offsets (if any)
  return {theta1 - dh_[0].theta, theta2 - dh_[1].theta, theta3 - dh_[2].theta};
}

// DH Transformation Matrix:
// This implements the standard Denavit–Hartenberg transformation:
// T_i = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
Eigen::Matrix4d KinematicsSolver::dhTransform(double theta, double d, double a,
                                              double alpha) {
  Eigen::Matrix4d T;

  // The matrix corresponds to:
  // [ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a*cos(θ) ]
  // [ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a*sin(θ) ]
  // [   0         sin(α)          cos(α)         d    ]
  // [   0           0               0            1    ]
  T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
      a * cos(theta), sin(theta), cos(theta) * cos(alpha),
      -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d, 0,
      0, 0, 1;

  return T;
}

bool KinematicsSolver::isReachable(const Eigen::Vector3d &footPos) const {
  if (dh_.size() != 3)
    return false;

  double x = footPos[0];
  double y = footPos[1];
  double z = footPos[2];

  double l1 = std::sqrt(std::pow(dh_[0].a, 2) + std::pow(dh_[0].d, 2));
  double l2 = std::sqrt(std::pow(dh_[1].a, 2) + std::pow(dh_[1].d, 2));
  double l3 = std::sqrt(std::pow(dh_[2].a, 2) + std::pow(dh_[2].d, 2));

  double x1 = std::sqrt(x * x + y * y) - l1;
  double d = std::sqrt(x1 * x1 + z * z);

  double cos_theta3 = (l2 * l2 + l3 * l3 - d * d) / (2 * l2 * l3);
  printf("cos_theta3=%.2f\n", cos_theta3);
  return cos_theta3 >= -1.0 && cos_theta3 <= 1.0;
}
