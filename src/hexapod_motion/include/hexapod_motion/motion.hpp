#ifndef MOTION_HPP
#define MOTION_HPP

#include "hexapod_msgs/msg/pose.hpp"
#include <string>
#include <vector>

struct Motion {
  std::string name;
  std::string category;
  std::string type;
  double duration;
  std::vector<hexapod_msgs::msg::Pose> poses;
};

#endif // !MOTION_HPP
