#ifndef HEXAPOD_RVIZ_PLUGINS_UI_POSE_LIST
#define HEXAPOD_RVIZ_PLUGINS_UI_POSE_LIST

#include <QListWidget>
#include <QStringList>
#include <hexapod_msgs/srv/command.hpp>
#include <qapplication.h>
#include <qinputdialog.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace hexapod_rviz_plugins {
class PoseList : public QListWidget {
  Q_OBJECT
  // In your class header
private:
  bool selecting_loop_ = false;
  QListWidgetItem *loop_start_item_ = nullptr;
  QListWidgetItem *loop_end_item_ = nullptr;
  QCursor loop_cursor_ = QCursor(Qt::CrossCursor); // customize as needed
  QIcon loop_start_icon_;
  QIcon loop_end_icon_;

public:
  PoseList();
  void removePose(size_t idx);
  void addPose();
  void moveCurrentPose(int distance = 1);
  void moveCurrentPoseUp();
  void moveCurrentPoseDown();

signals:
  void poseSelected(const size_t idx);

private:
  void onPoseSelected();
  void onRenamePose(QListWidgetItem *item);
  void startLoopSelection();
  void onItemClicked(QListWidgetItem *item);
  bool eventFilter(QObject *obj, QEvent *event);
  void cancelLoopSelection();
};
} // namespace hexapod_rviz_plugins
#endif
