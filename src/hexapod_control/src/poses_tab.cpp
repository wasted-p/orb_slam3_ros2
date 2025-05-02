#include "hexapod_control/poses_tab.hpp"
#include "hexapod_control/warehouse.hpp"
#include <qchar.h>
#include <qglobal.h>
#include <qmap.h>
#include <qsqlquery.h>
#include <qvariant.h>

PosesTab::PosesTab(QWidget *parent) : QWidget(parent) {
  // Initialize UI components
  table_ = new QTableWidget(this);
  create_button_ = new QPushButton("Create", this);
  delete_button_ = new QPushButton("Delete", this);
  save_button_ = new QPushButton("Save", this);
  move_up_button_ = new QPushButton("Move Up", this);
  move_down_button_ = new QPushButton("Move Down", this);

  // Setup layout
  QVBoxLayout *layout = new QVBoxLayout(this);
  QHBoxLayout *button_layout = new QHBoxLayout;
  button_layout->addWidget(create_button_);
  button_layout->addWidget(delete_button_);
  button_layout->addWidget(save_button_);
  button_layout->addWidget(move_up_button_);
  button_layout->addWidget(move_down_button_);
  layout->addWidget(table_);
  layout->addLayout(button_layout);
  setLayout(layout);

  node_ = NodeManager::getNode();

  sub_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&PosesTab::jointStateCallback, this, std::placeholders::_1));

  // Setup table
  setupTable();

  // Connect signals
  connect(create_button_, &QPushButton::clicked, this,
          &PosesTab::onCreateButtonClicked);
  connect(delete_button_, &QPushButton::clicked, this,
          &PosesTab::onDeleteButtonClicked);
  connect(save_button_, &QPushButton::clicked, this,
          &PosesTab::onSaveButtonClicked);
  connect(move_up_button_, &QPushButton::clicked, this,
          &PosesTab::onMoveUpButtonClicked);
  connect(move_down_button_, &QPushButton::clicked, this,
          &PosesTab::onMoveDownButtonClicked);
}

void PosesTab::setupTable() {
  table_->setColumnCount(3);
  table_->setHorizontalHeaderLabels({"ID", "Name", "Index"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  // Enable editing
  table_->setEditTriggers(QAbstractItemView::DoubleClicked |
                          QAbstractItemView::EditKeyPressed);
}

void PosesTab::addRow(float id, const QString &name, float index) {
  int row = table_->rowCount();
  table_->insertRow(row);

  // ID (editable, float)
  QTableWidgetItem *id_item = new QTableWidgetItem(QString::number(id));
  id_item->setData(Qt::EditRole, id); // Store as float
  table_->setItem(row, 0, id_item);

  // Name (non-editable, string)
  QTableWidgetItem *name_item = new QTableWidgetItem(name);
  table_->setItem(row, 1, name_item);

  // Index (editable, float)
  QTableWidgetItem *index_item = new QTableWidgetItem(QString::number(index));
  index_item->setData(Qt::EditRole, index); // Store as float
  table_->setItem(row, 2, index_item);
}

void PosesTab::onCreateButtonClicked() {
  // Add a new row with default values
  const int index = table_->rowCount() + 1.0f;
  addRow(index, QString("Pose%1").arg(table_->rowCount() + 1), 0.0f);

  auto &conn = WarehouseConnection::getInstance("warehouse.db");

  if (last_joint_msg_ == nullptr) {
    qDebug() << "No joint state message received yet!";
    return;
  }

  QMap<QString, QVariant> newPose;
  newPose["name"] = "Pose1";
  for (size_t i = 0; i < last_joint_msg_->name.size(); ++i) {
    newPose[last_joint_msg_->name[i].c_str()] =
        (i < last_joint_msg_->position.size() ? last_joint_msg_->position[i]
                                              : 0.0);
  }

  conn.insert("pose", newPose);
}

void PosesTab::onDeleteButtonClicked() {
  // Delete selected rows
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (auto it = ranges.rbegin(); it != ranges.rend(); ++it) {
    for (int row = it->bottomRow(); row >= it->topRow(); --row) {
      table_->removeRow(row);
    }
  }
}

void PosesTab::onSaveButtonClicked() {
  // Placeholder: Save table data (e.g., to file or database)
  qDebug() << "Saving table data:";
  for (int row = 0; row < table_->rowCount(); ++row) {
    float id = table_->item(row, 0)->data(Qt::EditRole).toFloat();
    QString name = table_->item(row, 1)->text();
    float index = table_->item(row, 2)->data(Qt::EditRole).toFloat();
    qDebug() << "Row" << row << ": ID =" << id << ", Name =" << name
             << ", Index =" << index;
  }
}

void PosesTab::onMoveUpButtonClicked() {
  // Move selected rows up
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (const auto &range : ranges) {
    for (int row = range.topRow(); row <= range.bottomRow(); ++row) {
      if (row > 0) {
        // Swap rows
        for (int col = 0; col < table_->columnCount(); ++col) {
          QTableWidgetItem *item_above = table_->takeItem(row - 1, col);
          QTableWidgetItem *item_current = table_->takeItem(row, col);
          table_->setItem(row - 1, col, item_current);
          table_->setItem(row, col, item_above);
        }
      }
    }
  }
  // Update selection
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
  // Move selected rows down
  QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
  for (auto it = ranges.rbegin(); it != ranges.rend(); ++it) {
    for (int row = it->bottomRow(); row >= it->topRow(); --row) {
      if (row < table_->rowCount() - 1) {
        // Swap rows
        for (int col = 0; col < table_->columnCount(); ++col) {
          QTableWidgetItem *item_below = table_->takeItem(row + 1, col);
          QTableWidgetItem *item_current = table_->takeItem(row, col);
          table_->setItem(row + 1, col, item_current);
          table_->setItem(row, col, item_below);
        }
      }
    }
  }
  // Update selection
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

void PosesTab::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Process joint state messages
  if (!msg->name.empty() && !msg->position.empty()) {
    std::string joint_info =
        "Joint: " + msg->name[0] + " Pos: " + std::to_string(msg->position[0]);

    last_joint_msg_ = msg;
  }
}
