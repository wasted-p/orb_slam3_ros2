#include <kinematics/solver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <urdf/model.h>

class URDFParser {
public:
  URDFParser() {}

  static std::vector<DHParam> getDHParams(const std::string &urdf_string,
                                          const std::string &base_link) {
    urdf::Model model;
    if (!model.initString(urdf_string))
      throw std::runtime_error("Failed to parse URDF");

    std::vector<DHParam> dh_params;
    std::string current_link = "base_link";

    Eigen::Vector3d prev_pos;
    bool has_prev = false;

    // Store joint origins for distance calculation
    std::vector<Eigen::Vector3d> joint_positions;
    std::vector<urdf::JointSharedPtr> joints;

    for (int i = 0; i < 3; ++i) {
      bool found = false;

      for (const auto &joint_pair : model.joints_) {
        auto joint = joint_pair.second;
        if (joint->parent_link_name == current_link) {
          joints.push_back(joint);

          auto pos = joint->parent_to_joint_origin_transform.position;
          joint_positions.emplace_back(pos.x, pos.y, pos.z);

          current_link = joint->child_link_name;
          found = true;
          break;
        }
      }

      if (!found) {
        throw std::runtime_error("Could not find joint with parent: " +
                                 current_link);
      }
    }

    // Now compute DH parameters
    for (size_t i = 0; i < joints.size(); ++i) {
      DHParam dh;

      // Use distance between this joint and the next (if available)
      if (i < joints.size() - 1) {
        dh.a = (joint_positions[i + 1] - joint_positions[i]).norm();
      } else {
        // For the last joint, fallback or assume small constant
        dh.a = 0.05; // or estimate from geometry, if needed
      }

      auto rot = joints[i]->parent_to_joint_origin_transform.rotation;

      dh.d = joints[i]->parent_to_joint_origin_transform.position.z;
      dh.alpha = std::atan2(rot.y * rot.z + rot.w * rot.x,
                            0.5 - (rot.x * rot.x + rot.y * rot.y));
      dh.theta = 0.0;

      dh_params.push_back(dh);
    }
    return dh_params;
  }
};
