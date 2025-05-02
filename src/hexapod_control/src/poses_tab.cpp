#include "hexapod_control/poses_tab.hpp"
#include "hexapod_control/warehouse.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <qabstractitemmodel.h>
#include <qchar.h>
#include <qglobal.h>
#include <qicon.h>
#include <qlist.h>
#include <qmap.h>
#include <qsqlquery.h>
#include <qtableview.h>
#include <qtablewidget.h>
#include <qvariant.h>
#include <string>
#include <vector>

PosesTab::PosesTab(QWidget *parent) : QWidget(parent) {
  // Initialize UI components
  table_ = new QTableWidget(this);
  add_button_ = new QPushButton("Add", this);
  delete_button_ = new QPushButton("Delete", this);
  save_button_ = new QPushButton("Save", this);
  move_up_button_ = new QPushButton("Move Up", this);
  move_down_button_ = new QPushButton("Move Down", this);

  // Setup layout
  QVBoxLayout *layout = new QVBoxLayout(this);
  QHBoxLayout *button_layout = new QHBoxLayout;
  button_layout->addWidget(add_button_);
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

  pub_joint_states_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  // Setup table
  setupTable();

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
  connect(table_, &QTableWidget::currentItemChanged, this,
          &PosesTab::onPoseSelected);
}

void PosesTab::onPoseSelected(QTableWidgetItem *current,
                              QTableWidgetItem *previous) {
  const int rowIdx = current->row();

  qDebug() << "Row" << rowIdx << "was clicked";

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->get_clock()->now();
  for (int column = 0; column < table_->columnCount() - 1; column++) {
    std::string joint_name =
        table_->horizontalHeaderItem(column)->text().toStdString();
    QTableWidgetItem *item = table_->item(rowIdx, column + 1);
    msg.name.push_back(joint_name);
    msg.position.push_back(item->text().toDouble());
    msg.velocity.push_back(0);
    msg.effort.push_back(0);
  }

  pub_joint_states_->publish(msg);
};
void PosesTab::setupTable() {
  table_->setColumnCount(22);

  // TODO: Put this in a global variable
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
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  // Enable editing
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers); // Make read-only
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

void PosesTab::addRow(QMap<QString, QVariant> newPose) {
  int row = table_->rowCount();
  table_->insertRow(row);

  // Iterate through all columns in the table
  for (int col = 0; col < table_->columnCount(); ++col) {
    // Get the header text for this column
    QString headerText = table_->horizontalHeaderItem(col)->text();

    // Check if this key exists in our pose map
    if (newPose.contains(headerText)) {
      QVariant value = newPose[headerText];
      QTableWidgetItem *item = nullptr;

      // Handle different data types appropriately
      if (value.type() == QVariant::String) {
        item = new QTableWidgetItem(value.toString());
      } else if (value.type() == QVariant::Double) {
        // Format doubles to 4 decimal places
        item = new QTableWidgetItem(QString::number(value.toDouble(), 'f', 4));
      } else if (value.type() == QVariant::Int ||
                 value.type() == QVariant::LongLong) {
        // Handle integer types
        item = new QTableWidgetItem(QString::number(value.toLongLong()));
      } else if (value.type() == QVariant::Bool) {
        // Handle boolean values
        item = new QTableWidgetItem(value.toBool() ? "True" : "False");
      } else {
        // Default fallback for other types
        item = new QTableWidgetItem(value.toString());
      }

      // Set the item in the table
      table_->setItem(row, col, item);

      // Store original value as item data for sorting or other operations
      item->setData(Qt::UserRole, value);
    } else {
      // If the key doesn't exist in our map, add an empty cell
      table_->setItem(row, col, new QTableWidgetItem(""));
    }
  }

  // Automatically adjust row height if needed
  table_->resizeRowToContents(row);
}

void PosesTab::onAddButtonClicked() {
  // Add a new row with default values
  const int index = table_->rowCount() + 1.0f;

  QMap<QString, QVariant> newPose;
  newPose["name"] = "Pose1";
  for (size_t i = 0; i < last_joint_msg_->name.size(); ++i) {
    newPose[last_joint_msg_->name[i].c_str()] =
        (i < last_joint_msg_->position.size() ? last_joint_msg_->position[i]
                                              : 0.0);
  }
  addRow(newPose);

  if (last_joint_msg_ == nullptr) {
    qDebug() << "No joint state message received yet!";
    return;
  }
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
