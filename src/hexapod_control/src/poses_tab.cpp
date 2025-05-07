#include "hexapod_control/poses_tab.hpp"

// NewCycleDialog Implementation (unchanged)
NewCycleDialog::NewCycleDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle("New Cycle");
  QVBoxLayout *layout = new QVBoxLayout(this);
  nameEdit_ = new QLineEdit(this);
  nameEdit_->setPlaceholderText("Enter cycle name");
  QPushButton *okButton = new QPushButton("OK", this);
  QPushButton *cancelButton = new QPushButton("Cancel", this);

  layout->addWidget(nameEdit_);
  QHBoxLayout *buttonLayout = new QHBoxLayout;
  buttonLayout->addWidget(okButton);
  buttonLayout->addWidget(cancelButton);
  layout->addLayout(buttonLayout);

  connect(okButton, &QPushButton::clicked, this, &QDialog::accept);
  connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

QString NewCycleDialog::getCycleName() const {
  return nameEdit_->text().trimmed();
}

// JointAngleDelegate Implementation (unchanged)
JointAngleDelegate::JointAngleDelegate(QObject *parent)
    : QItemDelegate(parent) {}

QWidget *JointAngleDelegate::createEditor(QWidget *parent,
                                          const QStyleOptionViewItem &option,
                                          const QModelIndex &index) const {
  QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
  editor->setRange(-M_PI / 2, M_PI / 2);
  editor->setDecimals(4);
  editor->setSingleStep(0.01);
  editor->setAlignment(Qt::AlignRight);
  editor->setFocusPolicy(Qt::WheelFocus);
  editor->setFrame(false);
  return editor;
}

void JointAngleDelegate::setEditorData(QWidget *editor,
                                       const QModelIndex &index) const {
  QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox *>(editor);
  bool ok;
  double value = index.model()->data(index, Qt::EditRole).toDouble(&ok);
  spinBox->setValue(ok ? value : 0.0);
}

void JointAngleDelegate::setModelData(QWidget *editor,
                                      QAbstractItemModel *model,
                                      const QModelIndex &index) const {
  QDoubleSpinBox *spinBox = static_cast<QDoubleSpinBox *>(editor);
  spinBox->interpretText();
  double value = spinBox->value();
  model->setData(index, QString::number(value, 'f', 4), Qt::EditRole);
}

// PosesTab Implementation
PosesTab::PosesTab(QWidget *parent) : QWidget(parent) {
  // Initialize UI components
  table_ = new QTableWidget(this);
  add_button_ = new QPushButton("Add", this);
  delete_button_ = new QPushButton("Delete", this);
  save_button_ = new QPushButton("Save", this);
  move_up_button_ = new QPushButton("Move Up", this);
  move_down_button_ = new QPushButton("Move Down", this);
  cycle_combo_ = new QComboBox(this);
  add_cycle_button_ = new QPushButton("+", this);
  play_button_ = new QPushButton("Play", this);
  angleDelegate_ = new JointAngleDelegate(this);

  // Setup layout
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  QHBoxLayout *cycle_layout = new QHBoxLayout;
  cycle_layout->addWidget(cycle_combo_);
  cycle_layout->addWidget(add_cycle_button_);
  cycle_layout->addWidget(play_button_);
  QHBoxLayout *button_layout = new QHBoxLayout;
  button_layout->addWidget(add_button_);
  button_layout->addWidget(delete_button_);
  button_layout->addWidget(save_button_);
  button_layout->addWidget(move_up_button_);
  button_layout->addWidget(move_down_button_);
  main_layout->addLayout(cycle_layout);
  main_layout->addWidget(table_);
  main_layout->addLayout(button_layout);
  setLayout(main_layout);

  // Initialize ROS node
  node_ = NodeManager::getNode();
  sub_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&PosesTab::jointStateCallback, this, std::placeholders::_1));
  pub_joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10).reliable());

  // Initialize trajectory action client
  trajectory_action_client_ =
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
          node_, "/legs_joint_trajectory_controller/follow_joint_trajectory");

  // Setup table
  setupTable();

  // Load cycles from database
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  QSqlQuery query = db.executeQuery("SELECT name FROM cycle");
  QStringList cycleNames;
  while (query.next()) {
    QString cycleName = query.value(0).toString();
    cycleNames << cycleName;
    cycles_[cycleName] = QList<QMap<QString, QVariant>>();
  }

  // If no cycles in database, create a default "base" cycle
  if (cycleNames.isEmpty()) {
    cycleNames << "base";
    cycles_["base"] = QList<QMap<QString, QVariant>>();
    QMap<QString, QVariant> initialPose;
    initialPose["name"] = "InitialPose";
    for (const auto &mapping : jointMappings_) {
      initialPose[mapping.actualName] = 0.0;
    }
    cycles_["base"].append(initialPose);
  }

  // Populate combo box
  cycle_combo_->addItems(cycleNames);
  cycle_combo_->setCurrentText(cycleNames.first());

  // Load poses for the first cycle
  updateTableForCycle(cycleNames.first());

  // Connect signals
  connect(add_button_, &QPushButton::clicked, this,
          &PosesTab::onAddButtonClicked);
  connect(delete_button_, &QPushButton::clicked, this,
          &PosesTab::onDeleteButtonClicked);
  connect(save_button_, &QPushButton::clicked, this,
          &PosesTab::onSaveButtonClicked);
  connect(move_up_button_, &QPushButton::clicked, this,
          &PosesTab::onMoveUpButtonClicked);
  connect(move_down_button_, &QPushButton::clicked, this,
          &PosesTab::onMoveDownButtonClicked);
  connect(cycle_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &PosesTab::onCycleChanged);
  connect(add_cycle_button_, &QPushButton::clicked, this,
          &PosesTab::onAddCycleClicked);
  connect(play_button_, &QPushButton::clicked, this,
          &PosesTab::onPlayButtonClicked);
  connect(table_, &QTableWidget::currentItemChanged, this,
          &PosesTab::onPoseSelected);
  connect(table_, &QTableWidget::cellChanged, this, &PosesTab::onCellChanged);

  // Send initial joint state message with all zeros
  publishInitialJointState();
}

// Define joint mappings
const QList<JointMapping> PosesTab::jointMappings_ = {
    {"Arm Rotator", "arm_rotator_joint"},
    {"Arm Abductor", "arm_abductor_joint"},
    {"Arm Retractor", "arm_retractor_joint"},
    {"Base Left Top Rotator", "top_left_rotate_joint"},
    {"Base Left Top Abductor", "top_left_abduct_joint"},
    {"Base Left Top Retractor", "top_left_retract_joint"},
    {"Base Left Mid Rotator", "mid_left_abduct_joint"},
    {"Base Left Mid Abductor", "mid_left_rotate_joint"},
    {"Base Left Mid Retractor", "mid_left_retract_joint"},
    {"Base Left Bottom Rotator", "bottom_left_rotate_joint"},
    {"Base Left Bottom Abductor", "bottom_left_abduct_joint"},
    {"Base Left Bottom Retractor", "bottom_left_retract_joint"},
    {"Base Right Top Rotator", "top_right_rotate_joint"},
    {"Base Right Top Abductor", "top_right_abduct_joint"},
    {"Base Right Top Retractor", "top_right_retract_joint"},
    {"Base Right Mid Rotator", "mid_right_rotate_joint"},
    {"Base Right Mid Abductor", "mid_right_abduct_joint"},
    {"Base Right Mid Retractor", "mid_right_retract_joint"},
    {"Base Right Bottom Rotator", "bottom_right_rotate_joint"},
    {"Base Right Bottom Abductor", "bottom_right_abduct_joint"},
    {"Base Right Bottom Retractor", "bottom_right_retract_joint"},
};

// Multilevel header structure
struct HeaderLevel {
  QString name;
  int span;
};

void PosesTab::setupTable() {
  // Set row count to number of joints + 1 for name
  table_->setRowCount(jointMappings_.size() + 1);
  table_->setColumnCount(1); // Will grow with poses

  // Setup vertical header labels
  QStringList verticalHeaders;
  verticalHeaders << "Pose Name";
  for (const auto &mapping : jointMappings_) {
    verticalHeaders << mapping.displayName;
  }
  table_->setVerticalHeaderLabels(verticalHeaders);

  // Setup multilevel horizontal header
  QHeaderView *header = table_->horizontalHeader();
  header->setSectionsMovable(false);
  header->setSectionResizeMode(QHeaderView::Fixed);

  // Configure scrolling
  table_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  table_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  table_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  // Set row heights
  table_->verticalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
  table_->setRowHeight(0, 30);
  for (int i = 1; i < table_->rowCount(); ++i) {
    table_->verticalHeader()->setSectionResizeMode(i, QHeaderView::Fixed);
    table_->setRowHeight(i, 30);
  }

  // Configure table properties
  table_->setSelectionBehavior(QAbstractItemView::SelectColumns);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::DoubleClicked |
                          QAbstractItemView::EditKeyPressed);
  table_->verticalHeader()->setStretchLastSection(false);

  // Set delegate for joint angle rows
  for (int i = 1; i < table_->rowCount(); ++i) {
    table_->setItemDelegateForRow(i, angleDelegate_);
  }
}

void PosesTab::addRow(QMap<QString, QVariant> newPose) {
  int col = table_->columnCount();
  table_->insertColumn(col);

  // Set pose name
  QTableWidgetItem *nameItem = new QTableWidgetItem;
  nameItem->setText(newPose["name"].toString());
  nameItem->setFlags(nameItem->flags() & ~Qt::ItemIsEditable);
  table_->setItem(0, col, nameItem);

  // Set joint values
  for (int row = 1; row < table_->rowCount(); ++row) {
    QString displayName = table_->verticalHeaderItem(row)->text();
    QString actualName;
    for (const auto &mapping : jointMappings_) {
      if (mapping.displayName == displayName) {
        actualName = mapping.actualName;
        break;
      }
    }

    QTableWidgetItem *item = new QTableWidgetItem;
    if (newPose.contains(actualName)) {
      QVariant value = newPose[actualName];
      if (value.type() == QVariant::Double) {
        item->setText(QString::number(value.toDouble(), 'f', 4));
      } else {
        item->setText(value.toString());
      }
      item->setData(Qt::UserRole, value);
    }
    table_->setItem(row, col, item);
  }
  table_->resizeColumnToContents(col);

  // Store the pose in the current cycle
  QString currentCycle = cycle_combo_->currentText();
  cycles_[currentCycle].append(newPose);

  // Automatically select the new column
  table_->clearSelection();
  table_->setRangeSelected(
      QTableWidgetSelectionRange(0, col, table_->rowCount() - 1, col), true);
}

void PosesTab::onAddButtonClicked() {
  QMap<QString, QVariant> newPose;
  newPose["name"] = QString("Pose%1").arg(table_->columnCount() + 1);

  if (last_joint_msg_ && !last_joint_msg_->name.empty()) {
    for (size_t i = 0; i < last_joint_msg_->name.size(); ++i) {
      QString joint = QString::fromStdString(last_joint_msg_->name[i]);
      double pos = (i < last_joint_msg_->position.size())
                       ? last_joint_msg_->position[i]
                       : 0.0;
      pos = std::clamp(pos, MIN_ANGLE, MAX_ANGLE);
      newPose[joint] = pos;
    }
  }

  // Ensure all joints have values
  for (const auto &mapping : jointMappings_) {
    if (!newPose.contains(mapping.actualName)) {
      newPose[mapping.actualName] = 0.0;
    }
  }

  addRow(newPose);
  table_->scrollTo(table_->model()->index(0, table_->columnCount() - 1));
}

void PosesTab::onDeleteButtonClicked() {
  QString currentCycle = cycle_combo_->currentText();
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (auto it = ranges.rbegin(); it != ranges.rend(); ++it) {
    for (int col = it->rightColumn(); col >= it->leftColumn(); --col) {
      table_->removeColumn(col);
      if (col < cycles_[currentCycle].size()) {
        cycles_[currentCycle].removeAt(col);
      }
    }
  }
}

void PosesTab::onSaveButtonClicked() {
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");

  // Get current cycle
  QString currentCycle = cycle_combo_->currentText();

  // Insert or update cycle
  QMap<QString, QVariant> cycleValues;
  cycleValues["name"] = currentCycle;
  if (!db.insert("cycle", cycleValues)) {
    qDebug() << "Failed to save cycle" << currentCycle;
    return;
  }

  // Get the cycle ID
  QSqlQuery query = db.executeQuery(
      QString("SELECT id FROM cycle WHERE name = '%1'").arg(currentCycle));
  if (!query.next()) {
    qDebug() << "Failed to retrieve cycle ID for" << currentCycle;
    return;
  }
  int cycleId = query.value(0).toInt();

  // Clear existing poses for this cycle
  if (!db.remove("pose", QString("cycle_id = %1").arg(cycleId))) {
    qDebug() << "Failed to clear existing poses for cycle" << currentCycle;
    return;
  }

  // Insert poses
  for (int col = 0; col < table_->columnCount(); ++col) {
    QMap<QString, QVariant> poseValues;
    QString poseName = table_->item(0, col)->text();
    poseValues["cycle_id"] = cycleId;
    poseValues["name"] = poseName;

    // Add joint values
    for (int row = 1; row < table_->rowCount(); ++row) {
      QString displayName = table_->verticalHeaderItem(row)->text();
      QString actualName;
      for (const auto &mapping : jointMappings_) {
        if (mapping.displayName == displayName) {
          actualName = mapping.actualName;
          break;
        }
      }
      QTableWidgetItem *item = table_->item(row, col);
      double value =
          item && !item->text().isEmpty() ? item->text().toDouble() : 0.0;
      poseValues[actualName] = value;
    }

    if (!db.insert("pose", poseValues)) {
      qDebug() << "Failed to save pose" << poseName << "for cycle"
               << currentCycle;
      return;
    }
  }

  qDebug() << "Successfully saved cycle" << currentCycle << "with"
           << table_->columnCount() << "poses";
}

void PosesTab::onMoveUpButtonClicked() {
  QString currentCycle = cycle_combo_->currentText();
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (const auto &range : ranges) {
    if (range.leftColumn() > 0) {
      for (int col = range.leftColumn(); col <= range.rightColumn(); ++col) {
        for (int row = 0; row < table_->rowCount(); ++row) {
          auto *item_left = table_->takeItem(row, col - 1);
          auto *item_current = table_->takeItem(row, col);
          table_->setItem(row, col - 1, item_current);
          table_->setItem(row, col, item_left);
        }
        cycles_[currentCycle].swapItemsAt(col - 1, col);
      }
    }
  }

  table_->clearSelection();
  for (const auto &range : ranges) {
    if (range.leftColumn() > 0) {
      table_->setRangeSelected(QTableWidgetSelectionRange(
                                   range.topRow(), range.leftColumn() - 1,
                                   range.bottomRow(), range.rightColumn() - 1),
                               true);
    }
  }
}

void PosesTab::onMoveDownButtonClicked() {
  QString currentCycle = cycle_combo_->currentText();
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (auto it = ranges.rbegin(); it != ranges.rend(); ++it) {
    for (int col = it->rightColumn(); col >= it->leftColumn(); --col) {
      if (col < table_->columnCount() - 1) {
        for (int row = 0; row < table_->rowCount(); ++row) {
          auto *item_right = table_->takeItem(row, col + 1);
          auto *item_current = table_->takeItem(row, col);
          table_->setItem(row, col + 1, item_current);
          table_->setItem(row, col, item_right);
        }
        cycles_[currentCycle].swapItemsAt(col, col + 1);
      }
    }
  }

  table_->clearSelection();
  for (const auto &range : ranges) {
    if (range.rightColumn() < table_->columnCount() - 1) {
      table_->setRangeSelected(QTableWidgetSelectionRange(
                                   range.topRow(), range.leftColumn() + 1,
                                   range.bottomRow(), range.rightColumn() + 1),
                               true);
    }
  }
}

void PosesTab::onPoseSelected(QTableWidgetItem *current,
                              QTableWidgetItem *previous) {
  if (!current)
    return;
  publishRowJointStates(current->column());
}

void PosesTab::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg->name.empty() && !msg->position.empty()) {
    last_joint_msg_ = msg;
  }
}

void PosesTab::onCellChanged(int row, int column) {
  if (row == 0)
    return;

  QTableWidgetItem *item = table_->item(row, column);
  if (!item)
    return;

  bool ok;
  double value = item->text().toDouble(&ok);
  if (!ok)
    return;

  value = std::clamp(value, MIN_ANGLE, MAX_ANGLE);
  item->setText(QString::number(value, 'f', 4));

  // Update the pose in the current cycle
  QString currentCycle = cycle_combo_->currentText();
  if (column < cycles_[currentCycle].size()) {
    QString displayName = table_->verticalHeaderItem(row)->text();
    QString actualName;
    for (const auto &mapping : jointMappings_) {
      if (mapping.displayName == displayName) {
        actualName = mapping.actualName;
        break;
      }
    }
    cycles_[currentCycle][column][actualName] = value;
  }

  publishRowJointStates(column);
}

void PosesTab::publishRowJointStates(int col) {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->get_clock()->now();

  for (int row = 1; row < table_->rowCount(); ++row) {
    QString displayName = table_->verticalHeaderItem(row)->text();
    QString actualName;
    for (const auto &mapping : jointMappings_) {
      if (mapping.displayName == displayName) {
        actualName = mapping.actualName;
        break;
      }
    }
    QTableWidgetItem *item = table_->item(row, col);
    double value =
        item && !item->text().isEmpty() ? item->text().toDouble() : 0.0;
    value = std::clamp(value, MIN_ANGLE, MAX_ANGLE);

    msg.name.push_back(actualName.toStdString());
    msg.position.push_back(value);
    msg.velocity.push_back(0.0);
  }

  pub_joint_states_->publish(msg);
  qDebug() << "Published complete joint state for column" << col;
}

void PosesTab::publishInitialJointState() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->get_clock()->now();

  for (const auto &mapping : jointMappings_) {
    msg.name.push_back(mapping.actualName.toStdString());
    msg.position.push_back(0.0);
    msg.velocity.push_back(0.0);
  }

  pub_joint_states_->publish(msg);
  qDebug() << "Published initial joint state with all zeros";
}

void PosesTab::onCycleChanged(int index) {
  QString cycleName = cycle_combo_->itemText(index);
  updateTableForCycle(cycleName);
}

void PosesTab::onAddCycleClicked() {
  NewCycleDialog dialog(this);
  if (dialog.exec() == QDialog::Accepted) {
    QString cycleName = dialog.getCycleName();
    if (!cycleName.isEmpty() && !cycles_.contains(cycleName)) {
      cycles_[cycleName] = QList<QMap<QString, QVariant>>();
      cycle_combo_->addItem(cycleName);
      cycle_combo_->setCurrentText(cycleName);
      updateTableForCycle(cycleName);
    }
  }
}

void PosesTab::updateTableForCycle(const QString &cycleName) {
  // Clear the table and cycles_ for this cycle
  table_->setColumnCount(0);
  cycles_[cycleName].clear();

  // Get cycle ID
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  QSqlQuery cycleQuery = db.executeQuery(
      QString("SELECT id FROM cycle WHERE name = '%1'").arg(cycleName));
  if (!cycleQuery.next()) {
    qDebug() << "Cycle" << cycleName << "not found in database";
    return;
  }
  int cycleId = cycleQuery.value(0).toInt();

  // Fetch poses for the cycle
  QSqlQuery poseQuery = db.executeQuery(
      QString("SELECT * FROM pose WHERE cycle_id = %1").arg(cycleId));
  while (poseQuery.next()) {
    QMap<QString, QVariant> pose;
    pose["name"] = poseQuery.value("name").toString();

    // Add joint values
    for (const auto &mapping : jointMappings_) {
      pose[mapping.actualName] = poseQuery.value(mapping.actualName).toDouble();
    }

    cycles_[cycleName].append(pose);
    addRow(pose);
  }

  qDebug() << "Loaded" << cycles_[cycleName].size() << "poses for cycle"
           << cycleName;
}

void PosesTab::onPlayButtonClicked() {
  if (is_trajectory_executing_) {
    return;
  }

  // Set button to loading state
  is_trajectory_executing_ = true;
  play_button_->setText("Executing...");
  play_button_->setEnabled(false);

  sendJointTrajectory();
}

void PosesTab::sendJointTrajectory() {
  if (!trajectory_action_client_->wait_for_action_server(
          std::chrono::seconds(5))) {
    qDebug() << "Trajectory action server not available";
    is_trajectory_executing_ = false;
    play_button_->setText("Play");
    play_button_->setEnabled(true);
    return;
  }

  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
  auto &trajectory = goal_msg.trajectory;

  // Get current cycle
  QString currentCycle = cycle_combo_->currentText();
  const auto &poses = cycles_[currentCycle];

  // Set joint names (excluding arm joints)
  for (const auto &mapping : jointMappings_) {
    if (!mapping.actualName.startsWith("arm_")) {
      trajectory.joint_names.push_back(mapping.actualName.toStdString());
    }
  }

  // Create trajectory points from poses
  double time_from_start = 0.0;
  const double duration_per_pose = 2.0; // 2 seconds per pose

  for (int col = 0; col < table_->columnCount(); ++col) {
    trajectory_msgs::msg::JointTrajectoryPoint point;

    // Add positions for leg joints only
    for (int row = 1; row < table_->rowCount(); ++row) {
      QString displayName = table_->verticalHeaderItem(row)->text();
      QString actualName;
      for (const auto &mapping : jointMappings_) {
        if (mapping.displayName == displayName &&
            !mapping.actualName.startsWith("arm_")) {
          actualName = mapping.actualName;
          QTableWidgetItem *item = table_->item(row, col);
          double value =
              item && !item->text().isEmpty() ? item->text().toDouble() : 0.0;
          value = std::clamp(value, MIN_ANGLE, MAX_ANGLE);
          point.positions.push_back(value);
          break;
        }
      }
    }

    point.velocities.resize(point.positions.size(), 0.0);
    point.accelerations.resize(point.positions.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);

    trajectory.points.push_back(point);
    time_from_start += duration_per_pose;
  }

  // Send the goal
  auto send_goal_options = rclcpp_action::Client<
      control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&PosesTab::handleTrajectoryResult, this, std::placeholders::_1);

  trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
  qDebug() << "Sent joint trajectory goal with" << trajectory.points.size()
           << "points";
}

void PosesTab::handleTrajectoryResult(
    const rclcpp_action::ClientGoalHandle<
        control_msgs::action::FollowJointTrajectory>::WrappedResult &result) {
  is_trajectory_executing_ = false;
  play_button_->setText("Play");
  play_button_->setEnabled(true);

  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    qDebug() << "Trajectory execution succeeded";
    break;
  case rclcpp_action::ResultCode::ABORTED:
    qDebug() << "Trajectory execution aborted";
    break;
  case rclcpp_action::ResultCode::CANCELED:
    qDebug() << "Trajectory execution canceled";
    break;
  default:
    qDebug() << "Trajectory execution failed with unknown result code";
    break;
  }
}
