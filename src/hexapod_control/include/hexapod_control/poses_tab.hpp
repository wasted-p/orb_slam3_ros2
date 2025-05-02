#ifndef POSES_TAB_H
#define POSES_TAB_H

#include "hexapod_control/node_manager.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <QPushButton>
#include <QSqlDatabase>
#include <QSqlError>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QtGlobal>
#include <qwidget.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <QDebug>
#include <QHeaderView>
#include <QItemDelegate>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

class PosesTab : public QWidget {
  Q_OBJECT
public:
  explicit PosesTab(QWidget *parent = nullptr);

private slots:
  void onCreateButtonClicked();
  void onDeleteButtonClicked();
  void onSaveButtonClicked();
  void onMoveUpButtonClicked();
  void onMoveDownButtonClicked();

private:
  QTableWidget *table_;
  QPushButton *create_button_;
  QPushButton *delete_button_;
  QPushButton *save_button_;
  QPushButton *move_up_button_;
  QPushButton *move_down_button_;

  sensor_msgs::msg::JointState::SharedPtr last_joint_msg_ =
      std::make_shared<sensor_msgs::msg::JointState>();

  // ROS node
  rclcpp::Node::SharedPtr node_;

  // ROS Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      sub_joint_states_;

  void setupTable();
  void addRow(float id = 0.0f, const QString &name = "Pose",
              float index = 0.0f);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

#endif // POSES_TAB_H
