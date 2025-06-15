
#include <filesystem>
#include <fstream>
#include <hexapod_motion/motion.hpp>
#include <map>
#include <yaml-cpp/yaml.h>

void loadFromYaml(std::map<std::string, Motion> &motions) {
  std::string base_path = "/home/thurdparty/Code/hexapod-ros/src/"
                          "hexapod_bringup/config/";
  std::string definitions_dir = base_path + "definitions/";
  std::string main_filename = base_path + "motions.yml";

  // node_->get_parameter("motion_definitions_path", main_filename);
  if (main_filename.empty())
    throw std::runtime_error("No 'motions' parameter specified.");

  YAML::Node root = YAML::LoadFile(main_filename);

  for (const auto &motion_pair : root["motions"]) {
    Motion motion;
    std::string motion_id = motion_pair.first.as<std::string>();
    const YAML::Node &motion_node = motion_pair.second;

    motion.name = motion_node["name"].as<std::string>();
    motion.category = motion_node["category"].as<std::string>("gait");
    motion.duration = motion_node["duration"].as<std::float_t>(1.0);
    motion.type = motion_node["type"].as<std::string>("cyclic");
    motion.poses = {};

    // Load poses from separate file
    std::string pose_filename = motion_node["definition"].as<std::string>();
    YAML::Node poses_root = YAML::LoadFile(pose_filename);

    for (const auto &pose_node : poses_root["poses"]) {
      hexapod_msgs::msg::Pose pose_msg;
      pose_msg.name = pose_node["name"].as<std::string>();

      for (const auto &n : pose_node["names"]) {
        pose_msg.names.push_back(n.as<std::string>());
      }

      for (const auto &pos : pose_node["positions"]) {
        geometry_msgs::msg::Point p;
        p.x = pos["x"].as<double>();
        p.y = pos["y"].as<double>();
        p.z = pos["z"].as<double>();
        pose_msg.positions.push_back(p);
      }

      motion.poses.push_back(pose_msg);
    }

    motions[motion_id] = motion;
  }
}

void saveToYaml(std::map<std::string, Motion> &motions) {
  std::string base_path = "/home/thurdparty/Code/hexapod-ros/src/"
                          "hexapod_bringup/config/";
  std::string definitions_dir = base_path + "definitions/";
  std::string main_filename = base_path + "motions.yml";

  if (motions.empty()) {
    throw std::invalid_argument("Motions is empty");
    return;
  }

  // Create definitions directory if it doesn't exist
  std::filesystem::create_directory(definitions_dir);

  YAML::Node root;
  YAML::Node motionsnode;

  for (const auto &motion_pair : motions) {
    const std::string &motion_id = motion_pair.first;
    const Motion &motion = motion_pair.second;

    // Create main motion entry (without poses)
    YAML::Node motion_node;

    std::string pose_filename = definitions_dir + motion_id + ".yml";
    motion_node["name"] = motion.name;
    motion_node["category"] = motion.category;
    motion_node["duration"] = motion.duration;
    motion_node["type"] = motion.type;
    motion_node["definition"] = pose_filename;
    motionsnode[motion_id] = motion_node;

    // Save poses to separate file
    YAML::Node poses_root;
    YAML::Node poses_node;

    for (const auto &pose_msg : motion.poses) {
      YAML::Node pose_node;
      pose_node["name"] = pose_msg.name;

      YAML::Node names_node;
      for (const auto &name : pose_msg.names) {
        names_node.push_back(name);
      }
      pose_node["names"] = names_node;

      YAML::Node positions_node;
      for (const auto &position : pose_msg.positions) {
        YAML::Node pos_node;
        pos_node["x"] = position.x;
        pos_node["y"] = position.y;
        pos_node["z"] = position.z;
        positions_node.push_back(pos_node);
      }
      pose_node["positions"] = positions_node;
      poses_node.push_back(pose_node);
    }

    poses_root["poses"] = poses_node;

    // Write poses file
    std::ofstream pose_file(pose_filename);
    if (!pose_file.is_open()) {
      throw std::runtime_error("Failed to open pose file for writing: " +
                               pose_filename);
    }
    pose_file << poses_root;
    pose_file.close();
  }

  root["motions"] = motionsnode;

  // Write main motion definitions file
  std::ofstream file(main_filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file for writing: " +
                             main_filename);
  }
  file << root;
  file.close();
}
