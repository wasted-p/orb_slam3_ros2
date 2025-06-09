#ifndef HEXAPOD_RVIZ_PLUGINS_GAIT_PLANNER_HPP
#define HEXAPOD_RVIZ_PLUGINS_GAIT_PLANNER_HPP

#include "hexapod_msgs/msg/gait.hpp"
#include "hexapod_msgs/msg/pose.hpp"
#include "hexapod_msgs/srv/control_markers.hpp"
#include "hexapod_msgs/srv/get_pose.hpp"
#include <QListWidget>
#include <QStringList>
#include <hexapod_msgs/msg/gait.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <ui/pose_list.hpp>

namespace hexapod_rviz_plugins {

class ActionPlannerRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  ActionPlannerRvizPanel(QWidget *parent = nullptr);
  ~ActionPlannerRvizPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onAddPose();
  void onDeletePose();
  void onExport();
  void onLoad();
  void onRenamePose(QListWidgetItem *item);
  void onPoseMoved(const int from_idx, const int to_idx);

private:
  void setCurrentPose(const size_t idx);
  void setupUi();
  void setupROS();
  hexapod_msgs::msg::Pose getInitialPose();

  void onPoseUpdate(hexapod_msgs::msg::Pose pose);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<hexapod_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<hexapod_msgs::msg::Pose>::SharedPtr pose_sub_;
  PoseList *pose_list_widget_;
  QStringList leg_names_;

  hexapod_msgs::msg::Pose initial_pose;

  int current_pose = -1;
  int created_poses_count_ = 0;

  std::map<std::string, hexapod_msgs::msg::Gait> actions_;
  hexapod_msgs::msg::Gait action_;

  rclcpp::Client<hexapod_msgs::srv::ControlMarkers>::SharedPtr client_;
  rclcpp::Client<hexapod_msgs::srv::GetPose>::SharedPtr get_pose_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  void createLoop(const int from_idx, const int to_idx);
};

} // namespace hexapod_rviz_plugins

#endif // HEXAPOD_RVIZ_PLUGINS_GAIT_PLANNER_HPP
