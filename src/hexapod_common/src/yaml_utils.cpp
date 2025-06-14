#include <hexapod_common/yaml_utils.hpp>

void parseYaml(std::string &yaml_string,
               std::map<std::string, geometry_msgs::msg::Point> map) {
  try {
    YAML::Node root = YAML::Load(yaml_string);
    for (const auto &pair : root["initial_pose"]) {
      std::string joint_name = pair.first.as<std::string>();
      geometry_msgs::msg::Point position;
      position.x = pair.second["x"].as<double>();
      position.y = pair.second["y"].as<double>();
      position.z = pair.second["z"].as<double>();
      map[joint_name] = position;
    }
  } catch (const YAML::Exception &e) {
    throw std::runtime_error(e.what());
  }
}

void parseYamls(std::string &yaml_string, std::map<std::string, double> &map) {
  try {
    YAML::Node root = YAML::Load(yaml_string);
    for (const auto &pair : root["initial_positions"]) {
      std::string joint_name = pair.first.as<std::string>();
      double value = pair.second.as<double>();
      map[joint_name] = value;
    }
  } catch (const YAML::Exception &e) {
    throw std::runtime_error(e.what());
  }
}
