#include <fstream>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

class LinkLengthExtractor : public rclcpp::Node {
public:
  explicit LinkLengthExtractor(const std::string &urdf_path)
      : Node("link_length_extractor") {
    RCLCPP_DEBUG(this->get_logger(), "Loading URDF from: %s",
                 urdf_path.c_str());

    std::ifstream urdf_file(urdf_path);
    if (!urdf_file) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file.");
      rclcpp::shutdown();
      return;
    }

    std::string urdf_xml((std::istreambuf_iterator<char>(urdf_file)),
                         std::istreambuf_iterator<char>());

    urdf::Model model;
    if (!model.initString(urdf_xml)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
      rclcpp::shutdown();
      return;
    }

    YAML::Node yaml_out;

    for (const auto &joint_pair : model.joints_) {
      const auto &joint = joint_pair.second;

      if (!joint || joint->parent_link_name.empty() ||
          joint->child_link_name.empty()) {
        continue;
      }

      auto parent_link = model.getLink(joint->parent_link_name);
      auto child_link = model.getLink(joint->child_link_name);

      if (parent_link && child_link) {
        const auto &origin = joint->parent_to_joint_origin_transform;

        // Length = Euclidean distance from parent origin to joint origin
        double length = std::sqrt(origin.position.x * origin.position.x +
                                  origin.position.y * origin.position.y +
                                  origin.position.z * origin.position.z);

        YAML::Node joint_data;
        joint_data["length"] = length;

        // Store roll-pitch-yaw from rotation (convert from quaternion)
        double roll, pitch, yaw;
        origin.rotation.getRPY(roll, pitch, yaw);

        joint_data["rotation"]["roll"] = roll;
        joint_data["rotation"]["pitch"] = pitch;
        joint_data["rotation"]["yaw"] = yaw;

        yaml_out[joint->child_link_name] = joint_data;
      }
    }

    std::string yaml_file = "link_lengths.yaml";
    std::ofstream fout(yaml_file);
    fout << yaml_out;
    fout.close();

    RCLCPP_DEBUG(this->get_logger(), "Saved link lengths and rotations to %s",
                 yaml_file.c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: extract_lengths_node <path_to_urdf>" << std::endl;
    return 1;
  }

  auto node = std::make_shared<LinkLengthExtractor>(argv[1]);

  // Work is done in constructor; shutdown immediately
  rclcpp::shutdown();
  return 0;
}
