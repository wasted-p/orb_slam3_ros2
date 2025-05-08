#ifndef POSES_TAB_HPP
#define POSES_TAB_HPP

#include <QComboBox>
#include <QDebug>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QItemDelegate>
#include <QLineEdit>
#include <QList>
#include <QMap>
#include <QPushButton>
#include <QString>
#include <QTableWidget>
#include <QTableWidgetSelectionRange>
#include <QVBoxLayout>
#include <QVariant>
#include <QWidget>
#include <cmath>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hexapod_control/warehouse.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

// Dialog for entering new cycle name
class NewCycleDialog : public QDialog {
  Q_OBJECT
public:
  explicit NewCycleDialog(QWidget *parent = nullptr);
  QString getCycleName() const;

private:
  QLineEdit *nameEdit_;
};

// Forward declaration
class PosesTab;

// Custom delegate for joint angle validation using QDoubleSpinBox
class JointAngleDelegate : public QItemDelegate {
  Q_OBJECT
public:
  JointAngleDelegate(PosesTab *posesTab, QObject *parent = nullptr);
  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                        const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model,
                    const QModelIndex &index) const override;

private slots:
  void onValueChanged(double value);

signals:
  void commitData(QWidget *editor) const; // Added 'const' to fix error

private:
  PosesTab *posesTab_;
  mutable QDoubleSpinBox *currentEditor_;
  mutable QModelIndex currentIndex_;
};

class PosesTab : public QWidget {
  Q_OBJECT

public:
  explicit PosesTab(QWidget *parent = nullptr);
  void publishRowJointStates(int column);

private slots:
  void onAddButtonClicked();
  void onDeleteButtonClicked();
  void onMoveUpButtonClicked();
  void onMoveDownButtonClicked();
  void onPoseSelected(QTableWidgetItem *current, QTableWidgetItem *previous);
  void onCellChanged(int row, int column);
  void onCycleChanged(int index);
  void onAddCycleClicked();
  void onDeleteCycleClicked();
  void onRenameCycleClicked();
  void onPlayButtonClicked();

private:
  // Struct to map display names to actual joint names
  struct JointMapping {
    QString displayName;
    QString actualName;
  };

  void setupTable();
  void addRow(QMap<QString, QVariant> newPose, int index = -1);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishInitialJointState();
  void updateTableForCycle(const QString &cycleName);
  void sendJointTrajectory();
  void handleTrajectoryResult(
      const rclcpp_action::ClientGoalHandle<
          control_msgs::action::FollowJointTrajectory>::WrappedResult &result);
  void updatePoseIndices(const QString &cycleName);

  QTableWidget *table_;
  QPushButton *add_button_;
  QPushButton *delete_button_;
  QPushButton *move_up_button_;
  QPushButton *move_down_button_;
  QComboBox *cycle_combo_;
  QPushButton *add_cycle_button_;
  QPushButton *delete_cycle_button_;
  QPushButton *rename_cycle_button_;
  QPushButton *play_button_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      sub_joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_msg_;

  // Action client for joint trajectory
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
      trajectory_action_client_;

  // Constants for joint angle limits
  const double MIN_ANGLE = -M_PI / 2;
  const double MAX_ANGLE = M_PI / 2;

  JointAngleDelegate *angleDelegate_;

  // Cycle storage: map of cycle name to list of poses (each pose includes 'id')
  QMap<QString, QList<QMap<QString, QVariant>>> cycles_;
  // Map of cycle name to cycle ID
  QMap<QString, int> cycleIds_;

  // Flag to track if trajectory is executing
  bool is_trajectory_executing_ = false;

  // Static joint mappings (only leg joints)
  static const QList<JointMapping> jointMappings_;
};

#endif // POSES_TAB_HPP
