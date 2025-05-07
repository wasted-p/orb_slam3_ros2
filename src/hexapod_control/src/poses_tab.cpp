#include "hexapod_control/poses_tab.hpp"
#include "hexapod_control/node_manager.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <QAbstractItemModel>
#include <QDoubleSpinBox>
#include <QList>
#include <QMap>
#include <QScrollBar>
#include <QString>
#include <QVariant>
#include <string>

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
  angleDelegate_ = new JointAngleDelegate(this);

  // Setup layout
  QVBoxLayout *main_layout = new QVBoxLayout(this);
  QHBoxLayout *cycle_layout = new QHBoxLayout;
  cycle_layout->addWidget(cycle_combo_);
  cycle_layout->addWidget(add_cycle_button_);
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

  // Setup table
  setupTable();

  // Initialize cycles with default "base" cycle
  cycles_["base"] = QList<QMap<QString, QVariant>>();
  cycle_combo_->addItem("base");
  cycle_combo_->setCurrentText("base");

  // Add initial joint state to the table for base cycle
  QMap<QString, QVariant> initialPose;
  initialPose["name"] = "InitialPose";
  for (int col = 1; col < table_->columnCount(); ++col) {
    QString joint = table_->horizontalHeaderItem(col)->text();
    initialPose[joint] = 0.0;
  }
  cycles_["base"].append(initialPose);
  addRow(initialPose);

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
  connect(table_, &QTableWidget::currentItemChanged, this,
          &PosesTab::onPoseSelected);
  connect(table_, &QTableWidget::cellChanged, this, &PosesTab::onCellChanged);

  // Send initial joint state message with all zeros
  publishInitialJointState();
}

void PosesTab::setupTable() {
  table_->setColumnCount(22);
  table_->setHorizontalHeaderLabels({
      "name",
      "arm_rotator_joint",
      "arm_abductor_joint",
      "arm_retractor_joint",
      "top_left_rotate_joint",
      "top_left_abduct_joint",
      "top_left_retract_joint",
      "mid_left_rotate_joint",
      "mid_left_abduct_joint",
      "mid_left_retract_joint",
      "bottom_left_rotate_joint",
      "bottom_left_abduct_joint",
      "bottom_left_retract_joint",
      "top_right_rotate_joint",
      "top_right_abduct_joint",
      "top_right_retract_joint",
      "mid_right_rotate_joint",
      "mid_right_abduct_joint",
      "mid_right_retract_joint",
      "bottom_right_rotate_joint",
      "bottom_right_abduct_joint",
      "bottom_right_retract_joint",
  });

  // Configure scrolling
  table_->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  table_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  table_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  // Set column widths
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
  table_->setColumnWidth(0, 150);
  for (int i = 1; i < table_->columnCount(); ++i) {
    table_->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Fixed);
    table_->setColumnWidth(i, 100);
  }

  // Configure table properties
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::DoubleClicked |
                          QAbstractItemView::EditKeyPressed);
  table_->horizontalHeader()->setStretchLastSection(false);

  // Set delegate for joint angle columns
  for (int i = 1; i < table_->columnCount(); ++i) {
    table_->setItemDelegateForColumn(i, angleDelegate_);
  }
}

void PosesTab::addRow(QMap<QString, QVariant> newPose) {
  int row = table_->rowCount();
  table_->insertRow(row);

  for (int col = 0; col < table_->columnCount(); ++col) {
    QString header = table_->horizontalHeaderItem(col)->text();
    QTableWidgetItem *item = new QTableWidgetItem;

    if (newPose.contains(header)) {
      QVariant value = newPose[header];
      if (value.type() == QVariant::Double) {
        item->setText(QString::number(value.toDouble(), 'f', 4));
      } else {
        item->setText(value.toString());
      }
      item->setData(Qt::UserRole, value);
    }

    table_->setItem(row, col, item);
    if (col == 0) {
      item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    }
  }
  table_->resizeRowToContents(row);

  // Store the pose in the current cycle
  QString currentCycle = cycle_combo_->currentText();
  cycles_[currentCycle].append(newPose);
}

void PosesTab::onAddButtonClicked() {
  QMap<QString, QVariant> newPose;
  newPose["name"] = QString("Pose%1").arg(table_->rowCount() + 1);

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
  for (int i = 1; i < table_->columnCount(); ++i) {
    QString joint = table_->horizontalHeaderItem(i)->text();
    if (!newPose.contains(joint)) {
      newPose[joint] = 0.0;
    }
  }

  addRow(newPose);
  table_->scrollToBottom();
}

void PosesTab::onDeleteButtonClicked() {
  QString currentCycle = cycle_combo_->currentText();
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (auto it = ranges.rbegin(); it != ranges.rend(); ++it) {
    for (int row = it->bottomRow(); row >= it->topRow(); --row) {
      table_->removeRow(row);
      if (row < cycles_[currentCycle].size()) {
        cycles_[currentCycle].removeAt(row);
      }
    }
  }
}

void PosesTab::onSaveButtonClicked() {
  qDebug() << "Saving poses for cycle:" << cycle_combo_->currentText();
  for (int row = 0; row < table_->rowCount(); ++row) {
    QString name = table_->item(row, 0)->text();
    qDebug() << "Pose:" << name;
    for (int col = 1; col < table_->columnCount(); ++col) {
      if (QTableWidgetItem *item = table_->item(row, col)) {
        QString joint = table_->horizontalHeaderItem(col)->text();
        qDebug() << "  " << joint << ":" << item->text();
      }
    }
  }
}

void PosesTab::onMoveUpButtonClicked() {
  QString currentCycle = cycle_combo_->currentText();
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (const auto &range : ranges) {
    if (range.topRow() > 0) {
      for (int row = range.topRow(); row <= range.bottomRow(); ++row) {
        for (int col = 0; col < table_->columnCount(); ++col) {
          auto *item_above = table_->takeItem(row - 1, col);
          auto *item_current = table_->takeItem(row, col);
          table_->setItem(row - 1, col, item_current);
          table_->setItem(row, col, item_above);
        }
        cycles_[currentCycle].swapItemsAt(row - 1, row);
      }
    }
  }

  table_->clearSelection();
  for (const auto &range : ranges) {
    if (range.topRow() > 0) {
      table_->setRangeSelected(QTableWidgetSelectionRange(
                                   range.topRow() - 1, range.leftColumn(),
                                   range.bottomRow() - 1, range.rightColumn()),
                               true);
    }
  }
}

void PosesTab::onMoveDownButtonClicked() {
  QString currentCycle = cycle_combo_->currentText();
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (auto it = ranges.rbegin(); it != ranges.rend(); ++it) {
    for (int row = it->bottomRow(); row >= it->topRow(); --row) {
      if (row < table_->rowCount() - 1) {
        for (int col = 0; col < table_->columnCount(); ++col) {
          auto *item_below = table_->takeItem(row + 1, col);
          auto *item_current = table_->takeItem(row, col);
          table_->setItem(row + 1, col, item_current);
          table_->setItem(row, col, item_below);
        }
        cycles_[currentCycle].swapItemsAt(row, row + 1);
      }
    }
  }

  table_->clearSelection();
  for (const auto &range : ranges) {
    if (range.bottomRow() < table_->rowCount() - 1) {
      table_->setRangeSelected(QTableWidgetSelectionRange(
                                   range.topRow() + 1, range.leftColumn(),
                                   range.bottomRow() + 1, range.rightColumn()),
                               true);
    }
  }
}

void PosesTab::onPoseSelected(QTableWidgetItem *current,
                              QTableWidgetItem *previous) {
  if (!current)
    return;
  publishRowJointStates(current->row());
}

void PosesTab::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!msg->name.empty() && !msg->position.empty()) {
    last_joint_msg_ = msg;
    qDebug() << "Received joint state with" << msg->name.size() << "joints";
  }
}

void PosesTab::onCellChanged(int row, int column) {
  if (column == 0)
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
  if (row < cycles_[currentCycle].size()) {
    QString joint = table_->horizontalHeaderItem(column)->text();
    cycles_[currentCycle][row][joint] = value;
  }

  publishRowJointStates(row);
}

void PosesTab::publishRowJointStates(int row) {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->get_clock()->now();

  for (int col = 1; col < table_->columnCount(); ++col) {
    QString joint = table_->horizontalHeaderItem(col)->text();
    QTableWidgetItem *item = table_->item(row, col);
    double value =
        item && !item->text().isEmpty() ? item->text().toDouble() : 0.0;
    value = std::clamp(value, MIN_ANGLE, MAX_ANGLE);

    msg.name.push_back(joint.toStdString());
    msg.position.push_back(value);
    msg.velocity.push_back(0.0);
    msg.effort.push_back(0.0);
  }

  pub_joint_states_->publish(msg);
  qDebug() << "Published complete joint state for row" << row;
}

void PosesTab::publishInitialJointState() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->get_clock()->now();

  for (int col = 1; col < table_->columnCount(); ++col) {
    QString joint = table_->horizontalHeaderItem(col)->text();
    msg.name.push_back(joint.toStdString());
    msg.position.push_back(0.0);
    msg.velocity.push_back(0.0);
    msg.effort.push_back(0.0);
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
  // Clear the table
  table_->setRowCount(0);

  // Populate with poses from the selected cycle
  const auto &poses = cycles_[cycleName];
  for (const auto &pose : poses) {
    addRow(pose);
  }
}
