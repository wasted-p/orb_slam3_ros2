
#include "hexapod_control/commands_tab.hpp"
#include <QDebug>
#include <QHeaderView>
#include <qcombobox.h>
#include <qlabel.h>
#include <qspinbox.h>

CommandsTab::CommandsTab(QWidget *parent) : QWidget(parent) {
  // Setup table
  setupUi();
  connectSlots();
}

void CommandsTab::connectSlots() {
  // // Connect signals
  // connect(stop_button_, &QPushButton::clicked, this,
  //         &CommandsTab::onStopButtonClicked);
  // connect(delete_button_, &QPushButton::clicked, this,
  //         &CommandsTab::onDeleteButtonClicked);
  // connect(save_button_, &QPushButton::clicked, this,
  //         &CommandsTab::onSaveButtonClicked);
  // connect(move_up_button_, &QPushButton::clicked, this,
  //         &CommandsTab::onMoveUpButtonClicked);
  // connect(move_down_button_, &QPushButton::clicked, this,
  //         &CommandsTab::onMoveDownButtonClicked);
}

void CommandsTab::setupUi() {
  // Commands Sub-tab
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);

  // Planning Group Dropdown
  planning_group_combo_ = new QComboBox;
  planning_group_combo_->addItem("manipulator");
  layout->addWidget(new QLabel("Planning Group:"));
  layout->addWidget(planning_group_combo_);

  // Planning Time Spin Box
  execute_time_spin_ = new QDoubleSpinBox;
  execute_time_spin_->setRange(0.1, 100.0);
  execute_time_spin_->setValue(5.0);
  execute_time_spin_->setSingleStep(0.1);
  layout->addWidget(new QLabel("Planning Time (s):"));
  layout->addWidget(execute_time_spin_);

  // Velocity Scaling Spin Box
  velocity_scaling_spin_ = new QDoubleSpinBox;
  velocity_scaling_spin_->setRange(0.0, 1.0);
  velocity_scaling_spin_->setValue(0.1);
  velocity_scaling_spin_->setSingleStep(0.05);
  layout->addWidget(new QLabel("Velocity Scaling:"));
  layout->addWidget(velocity_scaling_spin_);

  // Start State Dropdown
  start_pose_combo_ = new QComboBox;
  start_pose_combo_->addItem("<current>");
  layout->addWidget(new QLabel("Start Pose:"));
  layout->addWidget(start_pose_combo_);

  // Goal State Dropdown
  goal_pose_combo_ = new QComboBox;
  goal_pose_combo_->addItem("<current>");
  layout->addWidget(new QLabel("Goal Pose:"));
  layout->addWidget(goal_pose_combo_);

  // Buttons Layout
  QHBoxLayout *commands_buttons_layout = new QHBoxLayout;
  execute_button_ = new QPushButton("Execute");
  connect(execute_button_, SIGNAL(clicked()), this,
          SLOT(onExecuteButtonClicked()));
  commands_buttons_layout->addWidget(execute_button_);

  stop_button_ = new QPushButton("Stop");
  commands_buttons_layout->addWidget(stop_button_);

  layout->addLayout(commands_buttons_layout);

  // Checkboxes
  collision_aware_ik_cb_ = new QCheckBox("Collision-aware IK");

  layout->addWidget(collision_aware_ik_cb_);

  // Path Constraints Dropdown
  QComboBox *path_constraints_combo = new QComboBox;
  path_constraints_combo->addItem("NONE");
  layout->addWidget(new QLabel("Path Constraints:"));
  layout->addWidget(path_constraints_combo);
}

void CommandsTab::publishCommand(const std::string &command) {
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = command;
  pub_command_->publish(std::move(msg));
}

void CommandsTab::onExecuteButtonClicked() { publishCommand("execute"); }

void CommandsTab::onResetButtonClicked() { publishCommand("reset"); }
