#ifndef HEXAPOD_RVIZ_PLUGINS_GAIT_PLANNER_HPP
#define HEXAPOD_RVIZ_PLUGINS_GAIT_PLANNER_HPP

#include <QListWidget>
#include <QStringList>
#include <hexapod_msgs/srv/command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <ui/pose_list.hpp>

namespace hexapod_rviz_plugins {

class GaitPlannerRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  GaitPlannerRvizPanel(QWidget *parent = nullptr);
  ~GaitPlannerRvizPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void onAddPose();
  void onDeletePose();
  void onExport();
  void onLoad();
  void onRenamePose(QListWidgetItem *item);

private:
  void setCurrentPose(const size_t idx);
  void setupUi();
  void setupROS();

  rclcpp::Node::SharedPtr node_;
  PoseList *pose_list_widget_;
  QStringList leg_names_;
  rclcpp::Client<hexapod_msgs::srv::Command>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace hexapod_rviz_plugins

#endif // HEXAPOD_RVIZ_PLUGINS_GAIT_PLANNER_HPP
