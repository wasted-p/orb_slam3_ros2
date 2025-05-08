#include "hexapod_control/poses_tab.hpp"
#include "hexapod_control/node_manager.hpp"

// NewCycleDialog Implementation
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

// JointAngleDelegate Implementation
JointAngleDelegate::JointAngleDelegate(PosesTab *posesTab, QObject *parent)
    : QItemDelegate(parent), posesTab_(posesTab), currentEditor_(nullptr) {}

QWidget *JointAngleDelegate::createEditor(QWidget *parent,
                                          const QStyleOptionViewItem &option,
                                          const QModelIndex &index) const {
  QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
  editor->setRange(-M_PI / 2, M_PI / 2);
  editor->setDecimals(4);
  editor->setSingleStep(0.05);
  editor->setAlignment(Qt::AlignRight);
  editor->setFocusPolicy(Qt::WheelFocus);
  editor->setFrame(false);

  // Store editor and index for use in onValueChanged
  currentEditor_ = editor;
  currentIndex_ = index;

  // Connect valueChanged signal
  connect(editor, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &JointAngleDelegate::onValueChanged);

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

void JointAngleDelegate::onValueChanged(double value) {
  if (!currentEditor_ || !posesTab_)
    return;

  // Update table model immediately
  emit commitData(currentEditor_);
  // Publish joint states
  posesTab_->publishRowJointStates(currentIndex_.column());
}

// PosesTab Implementation
PosesTab::PosesTab(QWidget *parent) : QWidget(parent) {
  // Initialize UI components
  table_ = new QTableWidget(this);
  add_button_ = new QPushButton("Add", this);
  delete_button_ = new QPushButton("Delete", this);
  move_up_button_ = new QPushButton("Move Up", this);
  move_down_button_ = new QPushButton("Move Down", this);
  cycle_combo_ = new QComboBox(this);
  add_cycle_button_ = new QPushButton("+", this);
  delete_cycle_button_ = new QPushButton("−", this);
  rename_cycle_button_ = new QPushButton("Rename", this);
  play_button_ = new QPushButton("Play", this);
  angleDelegate_ = new JointAngleDelegate(this, this);

  // Setup layout
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  QHBoxLayout *cycle_layout = new QHBoxLayout;
  cycle_layout->addWidget(cycle_combo_);
  cycle_layout->addWidget(add_cycle_button_);
  cycle_layout->addWidget(delete_cycle_button_);
  cycle_layout->addWidget(rename_cycle_button_);
  cycle_layout->addWidget(play_button_);
  QHBoxLayout *button_layout = new QHBoxLayout;
  button_layout->addWidget(add_button_);
  button_layout->addWidget(delete_button_);
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

  // Initialize database and ensure pose table has pose_index column
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  QStringList createTableQueries;
  createTableQueries
      << "ALTER TABLE pose ADD COLUMN pose_index INTEGER DEFAULT 0";
  db.initializeTables(createTableQueries); // Idempotent if column exists

  // Load cycles from database
  QSqlQuery query = db.executeQuery("SELECT id, name FROM cycle");
  QStringList cycleNames;
  while (query.next()) {
    QString cycleName = query.value("name").toString();
    int cycleId = query.value("id").toInt();
    cycleNames << cycleName;
    cycleIds_[cycleName] = cycleId;
    cycles_[cycleName] = QList<QMap<QString, QVariant>>();
  }

  // If no cycles in database, create a default "base" cycle
  if (cycleNames.isEmpty()) {
    cycleNames << "base";
    cycles_["base"] = QList<QMap<QString, QVariant>>();
    QMap<QString, QVariant> cycleValues;
    cycleValues["name"] = "base";
    db.insert("cycle", cycleValues);
    QSqlQuery idQuery = db.executeQuery("SELECT last_insert_rowid()");
    if (idQuery.next()) {
      cycleIds_["base"] = idQuery.value(0).toInt();
    }
    QMap<QString, QVariant> initialPose;
    initialPose["name"] = "InitialPose";
    initialPose["cycle_id"] = cycleIds_["base"];
    initialPose["pose_index"] = 0;
    for (const auto &mapping : jointMappings_) {
      initialPose[mapping.actualName] = 0.0;
    }
    db.insert("pose", initialPose);
    QSqlQuery poseIdQuery = db.executeQuery("SELECT last_insert_rowid()");
    if (poseIdQuery.next()) {
      initialPose["id"] = poseIdQuery.value(0).toInt();
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
  connect(move_up_button_, &QPushButton::clicked, this,
          &PosesTab::onMoveUpButtonClicked);
  connect(move_down_button_, &QPushButton::clicked, this,
          &PosesTab::onMoveDownButtonClicked);
  connect(cycle_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &PosesTab::onCycleChanged);
  connect(add_cycle_button_, &QPushButton::clicked, this,
          &PosesTab::onAddCycleClicked);
  connect(delete_cycle_button_, &QPushButton::clicked, this,
          &PosesTab::onDeleteCycleClicked);
  connect(rename_cycle_button_, &QPushButton::clicked, this,
          &PosesTab::onRenameCycleClicked);
  connect(play_button_, &QPushButton::clicked, this,
          &PosesTab::onPlayButtonClicked);
  connect(table_, &QTableWidget::currentItemChanged, this,
          &PosesTab::onPoseSelected);
  connect(table_, &QTableWidget::cellChanged, this, &PosesTab::onCellChanged);

  // Send initial joint state message with all zeros
  publishInitialJointState();
}

// Define joint mappings (only leg joints)
const QList<PosesTab::JointMapping> PosesTab::jointMappings_ = {
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

  // Setup horizontal header
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

void PosesTab::addRow(QMap<QString, QVariant> newPose, int index) {
  int col = (index == -1) ? table_->columnCount() : index;
  if (index != -1) {
    table_->insertColumn(col);
  } else {
    table_->setColumnCount(table_->columnCount() + 1);
  }

  // Set pose name (editable)
  QTableWidgetItem *nameItem = new QTableWidgetItem;
  nameItem->setText(newPose["name"].toString());
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
  if (index != -1 && index < cycles_[currentCycle].size()) {
    cycles_[currentCycle].insert(index, newPose);
  } else {
    cycles_[currentCycle].append(newPose);
  }

  // Automatically select the new column
  table_->clearSelection();
  table_->setRangeSelected(
      QTableWidgetSelectionRange(0, col, table_->rowCount() - 1, col), true);
}

void PosesTab::onAddButtonClicked() {
  QMap<QString, QVariant> newPose;
  newPose["name"] = QString("Pose%1").arg(table_->columnCount() + 1);
  newPose["cycle_id"] = cycleIds_[cycle_combo_->currentText()];
  newPose["pose_index"] = cycles_[cycle_combo_->currentText()].size();

  if (last_joint_msg_ && !last_joint_msg_->name.empty()) {
    for (size_t i = 0; i < last_joint_msg_->name.size(); ++i) {
      QString joint = QString::fromStdString(last_joint_msg_->name[i]);
      // Only include leg joints
      for (const auto &mapping : jointMappings_) {
        if (mapping.actualName == joint) {
          double pos = (i < last_joint_msg_->position.size())
                           ? last_joint_msg_->position[i]
                           : 0.0;
          pos = std::clamp(pos, MIN_ANGLE, MAX_ANGLE);
          newPose[joint] = pos;
          break;
        }
      }
    }
  }

  // Ensure all leg joints have values
  for (const auto &mapping : jointMappings_) {
    if (!newPose.contains(mapping.actualName)) {
      newPose[mapping.actualName] = 0.0;
    }
  }

  // Insert into database
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  if (!db.insert("pose", newPose)) {

    qDebug() << "Failed to insert pose" << newPose["name"].toString();
    return;
  }

  // Fetch pose ID
  QSqlQuery idQuery = db.executeQuery("SELECT last_insert_rowid()");
  if (idQuery.next()) {
    newPose["id"] = idQuery.value(0).toInt();
  }

  addRow(newPose);
  table_->scrollTo(table_->model()->index(0, table_->columnCount() - 1));
}

void PosesTab::onDeleteButtonClicked() {
  QString currentCycle = cycle_combo_->currentText();
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();

  for (auto it = ranges.rbegin(); it != ranges.rend(); ++it) {
    for (int col = it->rightColumn(); col >= it->leftColumn(); --col) {
      if (col < cycles_[currentCycle].size()) {
        int poseId = cycles_[currentCycle][col]["id"].toInt();
        db.remove("pose", QString("id = %1").arg(poseId));
        cycles_[currentCycle].removeAt(col);
        table_->removeColumn(col);
      }
    }
  }

  // Update pose indices
  updatePoseIndices(currentCycle);
}

void PosesTab::onMoveUpButtonClicked() {
  QString currentCycle = cycle_combo_->currentText();
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");

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

  // Update pose indices
  updatePoseIndices(currentCycle);

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
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");

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

  // Update pose indices
  updatePoseIndices(currentCycle);

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

void PosesTab::updatePoseIndices(const QString &cycleName) {
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  for (int i = 0; i < cycles_[cycleName].size(); ++i) {
    QMap<QString, QVariant> &pose = cycles_[cycleName][i];
    int poseId = pose["id"].toInt();
    pose["pose_index"] = i;
    QMap<QString, QVariant> values;
    values["pose_index"] = i;
    db.update("pose", values, QString("id = %1").arg(poseId));
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
  QTableWidgetItem *item = table_->item(row, column);
  if (!item)
    return;

  QString currentCycle = cycle_combo_->currentText();
  if (column >= cycles_[currentCycle].size())
    return;

  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  int poseId = cycles_[currentCycle][column]["id"].toInt();

  if (row == 0) {
    // Handle pose name change
    QString newName = item->text().trimmed();
    if (!newName.isEmpty()) {
      cycles_[currentCycle][column]["name"] = newName;
      QMap<QString, QVariant> values;
      values["name"] = newName;
      db.update("pose", values, QString("id = %1").arg(poseId));
    } else {
      // Revert to original name if empty
      item->setText(cycles_[currentCycle][column]["name"].toString());
    }
  } else {
    // Handle joint angle change
    bool ok;
    double value = item->text().toDouble(&ok);
    if (!ok)
      return;

    value = std::clamp(value, MIN_ANGLE, MAX_ANGLE);
    item->setText(QString::number(value, 'f', 4));

    QString displayName = table_->verticalHeaderItem(row)->text();
    QString actualName;
    for (const auto &mapping : jointMappings_) {
      if (mapping.displayName == displayName) {
        actualName = mapping.actualName;
        break;
      }
    }
    cycles_[currentCycle][column][actualName] = value;

    // Update database
    QMap<QString, QVariant> values;
    values[actualName] = value;
    db.update("pose", values, QString("id = %1").arg(poseId));

    // Joint state publishing is handled by JointAngleDelegate::onValueChanged
  }
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
      WarehouseConnection &db =
          WarehouseConnection::getInstance("warehouse.db");
      QMap<QString, QVariant> cycleValues;
      cycleValues["name"] = cycleName;
      if (!db.insert("cycle", cycleValues)) {
        qDebug() << "Failed to insert cycle" << cycleName;
        return;
      }

      // Fetch cycle ID
      QSqlQuery idQuery = db.executeQuery("SELECT last_insert_rowid()");
      if (idQuery.next()) {
        cycleIds_[cycleName] = idQuery.value(0).toInt();
      }

      cycles_[cycleName] = QList<QMap<QString, QVariant>>();
      cycle_combo_->addItem(cycleName);
      cycle_combo_->setCurrentText(cycleName);
      updateTableForCycle(cycleName);
    }
  }
}

void PosesTab::onDeleteCycleClicked() {
  QString currentCycle = cycle_combo_->currentText();
  if (cycle_combo_->count() <= 1) {
    qDebug() << "Cannot delete the last cycle";
    return;
  }

  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  int cycleId = cycleIds_[currentCycle];

  // Delete poses and cycle from database
  db.remove("pose", QString("cycle_id = %1").arg(cycleId));
  db.remove("cycle", QString("id = %1").arg(cycleId));

  // Remove from memory
  cycles_.remove(currentCycle);
  cycleIds_.remove(currentCycle);

  // Update UI
  int currentIndex = cycle_combo_->currentIndex();
  cycle_combo_->removeItem(currentIndex);
  if (cycle_combo_->count() > 0) {
    updateTableForCycle(cycle_combo_->currentText());
  }
}

void PosesTab::onRenameCycleClicked() {
  QString currentCycle = cycle_combo_->currentText();
  NewCycleDialog dialog(this);
  dialog.setWindowTitle("Rename Cycle");
  if (dialog.exec() == QDialog::Accepted) {
    QString newCycleName = dialog.getCycleName();
    if (!newCycleName.isEmpty() && !cycles_.contains(newCycleName) &&
        newCycleName != currentCycle) {
      WarehouseConnection &db =
          WarehouseConnection::getInstance("warehouse.db");
      QMap<QString, QVariant> values;
      values["name"] = newCycleName;
      int cycleId = cycleIds_[currentCycle];
      if (!db.update("cycle", values, QString("id = %1").arg(cycleId))) {
        qDebug() << "Failed to rename cycle" << currentCycle << "to"
                 << newCycleName;
        return;
      }

      // Update memory
      cycles_[newCycleName] = cycles_.take(currentCycle);
      cycleIds_[newCycleName] = cycleIds_.take(currentCycle);

      // Update UI
      int currentIndex = cycle_combo_->currentIndex();
      cycle_combo_->setItemText(currentIndex, newCycleName);
      cycle_combo_->setCurrentText(newCycleName);
    }
  }
}

void PosesTab::updateTableForCycle(const QString &cycleName) {
  // Clear the table and cycles_ for this cycle
  table_->setColumnCount(0);
  cycles_[cycleName].clear();

  // Get cycle ID
  WarehouseConnection &db = WarehouseConnection::getInstance("warehouse.db");
  int cycleId = cycleIds_[cycleName];

  // Fetch poses for the cycle, sorted by pose_index
  QSqlQuery poseQuery = db.executeQuery(
      QString("SELECT * FROM pose WHERE cycle_id = %1 ORDER BY pose_index")
          .arg(cycleId));
  while (poseQuery.next()) {
    QMap<QString, QVariant> pose;
    pose["id"] = poseQuery.value("id").toInt();
    pose["name"] = poseQuery.value("name").toString();
    pose["cycle_id"] = cycleId;
    pose["pose_index"] = poseQuery.value("pose_index").toInt();

    // Add leg joint values
    for (const auto &mapping : jointMappings_) {
      pose[mapping.actualName] = poseQuery.value(mapping.actualName).toDouble();
    }

    cycles_[cycleName].append(pose);
    addRow(pose, cycles_[cycleName].size() - 1);
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

  // Set joint names (all are leg joints)
  for (const auto &mapping : jointMappings_) {
    trajectory.joint_names.push_back(mapping.actualName.toStdString());
  }

  // Create trajectory points from poses
  double time_from_start = 0.0;
  const double duration_per_pose = 2.0; // 2 seconds per pose

  for (int col = 0; col < table_->columnCount(); ++col) {
    trajectory_msgs::msg::JointTrajectoryPoint point;

    // Add positions for leg joints
    for (int row = 1; row < table_->rowCount(); ++row) {
      QString displayName = table_->verticalHeaderItem(row)->text();
      QString actualName;
      for (const auto &mapping : jointMappings_) {
        if (mapping.displayName == displayName) {
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
