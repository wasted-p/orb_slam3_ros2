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

#include "hexapod_control/warehouse.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Dialog for entering new cycle name
class NewCycleDialog : public QDialog {
  Q_OBJECT
public:
  explicit NewCycleDialog(QWidget *parent = nullptr);
  QString getCycleName() const;

private:
  QLineEdit *nameEdit_;
};

// Custom delegate for joint angle validation using QDoubleSpinBox
class JointAngleDelegate : public QItemDelegate {
public:
  JointAngleDelegate(QObject *parent = nullptr);
  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option,
                        const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model,
                    const QModelIndex &index) const override;
};

class PosesTab : public QWidget {
  Q_OBJECT

public:
  explicit PosesTab(QWidget *parent = nullptr);

private slots:
  void onAddButtonClicked();
  void onDeleteButtonClicked();
  void onSaveButtonClicked();
  void onMoveUpButtonClicked();
  void onMoveDownButtonClicked();
  void onPoseSelected(QTableWidgetItem *current, QTableWidgetItem *previous);
  void onCellChanged(int row, int column);
  void onCycleChanged(int index);
  void onAddCycleClicked();

private:
  void setupTable();
  void addRow(QMap<QString, QVariant> newPose);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publishRowJointStates(int row);
  void publishInitialJointState();
  void updateTableForCycle(const QString &cycleName);

  QTableWidget *table_;
  QPushButton *add_button_;
  QPushButton *delete_button_;
  QPushButton *save_button_;
  QPushButton *move_up_button_;
  QPushButton *move_down_button_;
  QComboBox *cycle_combo_;
  QPushButton *add_cycle_button_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      sub_joint_states_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_msg_;

  // Constants for joint angle limits
  const double MIN_ANGLE = -M_PI / 2;
  const double MAX_ANGLE = M_PI / 2;

  JointAngleDelegate *angleDelegate_;

  // Cycle storage: map of cycle name to list of poses
  QMap<QString, QList<QMap<QString, QVariant>>> cycles_;
};

#endif // POSES_TAB_HPP
