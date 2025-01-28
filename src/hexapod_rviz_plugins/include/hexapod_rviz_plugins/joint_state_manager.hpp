#ifndef HEXAPOD_RVIZ_PLUGINS__JOINT_STATE_DISPLAY_HPP_
#define HEXAPOD_RVIZ_PLUGINS__JOINT_STATE_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hexapod_rviz_plugins
{
class JointStateManager
  : public rviz_common::MessageFilterDisplay<sensor_msgs::msg::JointState>
{
  Q_OBJECT

protected:
  // Override the processMessage function to handle JointState messages
  void processMessage(const sensor_msgs::msg::JointState::ConstSharedPtr msg) override;
};
}  // namespace hexapod_rviz_plugins

#endif  // HEXAPOD_RVIZ_PLUGINS__JOINT_STATE_DISPLAY_HPP_
