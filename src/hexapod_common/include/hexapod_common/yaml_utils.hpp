#ifndef HEXAPOD_COMMON_YAML_UTILS_HPP
#define HEXAPOD_COMMON_YAML_UTILS_HPP

#include "geometry_msgs/msg/point.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include <map>
#include <string>

#include <filesystem>
#include <fstream>
#include <map>
#include <yaml-cpp/yaml.h>

struct Motion {
  std::string name;
  std::string category;
  std::string type;
  double duration;
  std::vector<hexapod_msgs::msg::Pose> poses;
};

void parseYaml(std::string &yaml_string,
               std::map<std::string, geometry_msgs::msg::Point> &map);

void parseYamls(std::string &yaml_string, std::map<std::string, double> &map);

template <typename... Args> std::string joinWithSlash(Args &&...args) {
  std::vector<std::string> parts{std::forward<Args>(args)...};
  std::ostringstream oss;
  bool first = true;
  for (const auto &part : parts) {
    if (part.empty())
      continue;
    if (!first)
      oss << "/";
    oss << part;
    first = false;
  }
  return oss.str();
}

void loadFromYaml(std::map<std::string, Motion> &motions);

void saveToYaml(std::map<std::string, Motion> &motions);
#endif
