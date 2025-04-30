#ifndef COMMANDS_TAB_H
#define COMMANDS_TAB_H

#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>

#include <QItemDelegate>
#include <qcheckbox.h>
#include <qcombobox.h>
#include <qlabel.h>
#include <qspinbox.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

const char DB_PATH[] = "/home/theycallmemuzz/Code/hexapod-ros/warehouse.db";

class CommandsTab : public QWidget {
  Q_OBJECT
public:
  explicit CommandsTab(QWidget *parent = nullptr);

protected Q_SLOTS:
  // Button click handlers
  void onExecuteButtonClicked();
  void onResetButtonClicked();
  void onCreateWarehouseButtonClicked(const char *dbName = DB_PATH);
  void onStopButtonClicked();

private:
  // ROS Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_command_;

  // ROS Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;

  QPushButton *execute_button_;
  QPushButton *stop_button_;
  QPushButton *reset_button_;
  QComboBox *planning_group_combo_;
  QDoubleSpinBox *execute_time_spin_;
  QDoubleSpinBox *velocity_scaling_spin_;
  QComboBox *start_pose_combo_;
  QComboBox *goal_pose_combo_;
  // Checkboxes
  QCheckBox *collision_aware_ik_cb_;

  void setupUi();
  void connectSlots();

  void publishCommand(const std::string &command);
};

#endif // POSES_TAB_H
