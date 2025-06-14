
#include "geometry_msgs/msg/point.hpp"
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>

void parseYaml(std::string &yaml_string,
               std::map<std::string, geometry_msgs::msg::Point> map);

void parseYamls(std::string &yaml_string, std::map<std::string, double> &map);
