#ifndef MOTION_YAML_HPP
#define MOTION_YAML_HPP

#include <filesystem>
#include <fstream>
#include <hexapod_motion/motion.hpp>
#include <map>
#include <yaml-cpp/yaml.h>

void loadFromYaml(std::map<std::string, Motion> &motions);

void saveToYaml(std::map<std::string, Motion> &motions);
#endif
