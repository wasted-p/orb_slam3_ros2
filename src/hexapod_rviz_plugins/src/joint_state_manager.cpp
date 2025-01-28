#include <hexapod_rviz_plugins/joint_state_manager.hpp>
#include <rviz_common/logging.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace hexapod_rviz_plugins
{
void JointStateManager::processMessage(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  // Log the header frame ID
  RVIZ_COMMON_LOG_INFO_STREAM("Received JointState message with frame: " << msg->header.frame_id);

  // Log joint names and positions
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    RVIZ_COMMON_LOG_INFO_STREAM(
      "Joint: " << msg->name[i]
                << ", Position: " << (i < msg->position.size() ? msg->position[i] : 0.0));
  }
}
}  // namespace hexapod_rviz_plugins

// Export the plugin so it can be loaded in RViz2
PLUGINLIB_EXPORT_CLASS(hexapod_rviz_plugins::JointStateManager, rviz_common::Display)
