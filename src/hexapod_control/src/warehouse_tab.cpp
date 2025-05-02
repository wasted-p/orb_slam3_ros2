#include "hexapod_control/warehouse_tab.hpp"
#include "hexapod_control/warehouse.hpp"
#include <QDebug>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QSqlDatabase>
#include <qfiledialog.h>
#include <qglobal.h>
#include <qlist.h>
#include <qmessagebox.h>

WarehouseTab::WarehouseTab(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);

  // Title label
  QLabel *displays_label = new QLabel("View Warehouse Connection");
  layout->addWidget(displays_label);

  // Database file path input
  QLabel *db_path_label = new QLabel("Database File Path:");
  QLineEdit *db_path_input_ = new QLineEdit;
  db_path_input_->setPlaceholderText(
      "Enter or select SQLite database file path");
  layout->addWidget(db_path_label);
  layout->addWidget(db_path_input_);

  // Browse button for selecting database file
  QPushButton *browse_button_ = new QPushButton("Browse...");
  layout->addWidget(browse_button_);

  // Connect and Disconnect buttons
  create_button_ = new QPushButton("Create");
  connect_button_ = new QPushButton("Connect");
  disconnect_button_ = new QPushButton("Disconnect");
  disconnect_button_->setEnabled(false); // Initially disabled until connected
  QHBoxLayout *button_layout = new QHBoxLayout;
  button_layout->addWidget(connect_button_);
  button_layout->addWidget(create_button_);
  button_layout->addWidget(disconnect_button_);
  layout->addLayout(button_layout);

  // Status label for connection feedback
  status_label_ = new QLabel("Status: Not Connected");
  layout->addWidget(status_label_);

  // Spacer to push content to the top
  layout->addStretch();

  // Setup layout
  // QVBoxLayout *layout = new QVBoxLayout(this);
  // QHBoxLayout *button_layout = new QHBoxLayout;
  // button_layout->addWidget(create_button__);
  // button_layout->addWidget(delete_button_);
  // button_layout->addWidget(save_button_);
  // button_layout->addWidget(move_up_button_);
  // button_layout->addWidget(move_down_button_);
  // layout->addWidget(table_);
  // layout->addLayout(button_layout);
  // setLayout(layout);

  // Setup table

  // Connect signals
  connect(create_button_, &QPushButton::clicked, this,
          &WarehouseTab::onCreateButtonClicked);

  connect(connect_button_, &QPushButton::clicked, this,
          &WarehouseTab::onConnectButtonClicked);

  connect(disconnect_button_, &QPushButton::clicked, this,
          &WarehouseTab::onDisconnectButtonClicked);

  connect(browse_button_, &QPushButton::clicked, this,
          &WarehouseTab::onBrowseButtonClicked);
}

void WarehouseTab::onCreateButtonClicked() {
  // Specify the database path (replace with your desired path)
  QString dbPath =
      "/home/theycallmemuzz/Code/hexapod-ros/src/hexapod_control/warehouse.db";

  auto &conn = WarehouseConnection::getInstance("warehouse.db");

  QStringList queries = {"CREATE TABLE cycle ("
                         "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                         "name TEXT"
                         ") ",
                         "CREATE TABLE pose ("
                         "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                         "cycle_id INTEGER, "
                         "name TEXT, "
                         "arm_rotator_joint REAL,"
                         "arm_abductor_joint REAL,"
                         "arm_retractor_joint REAL,"
                         "top_left_rotate_joint REAL,"
                         "top_left_abduct_joint REAL,"
                         "top_left_retract_joint REAL,"
                         "mid_left_rotate_joint REAL,"
                         "mid_left_abduct_joint REAL,"
                         "mid_left_retract_joint REAL,"
                         "bottom_left_rotate_joint REAL,"
                         "bottom_left_abduct_joint REAL,"
                         "bottom_left_retract_joint REAL,"
                         "top_right_rotate_joint REAL,"
                         "top_right_abduct_joint REAL,"
                         "top_right_retract_joint REAL,"
                         "mid_right_rotate_joint REAL,"
                         "mid_right_abduct_joint REAL,"
                         "mid_right_retract_joint REAL,"
                         "bottom_right_rotate_joint REAL,"
                         "bottom_right_abduct_joint REAL,"
                         "bottom_right_retract_joint REAL,"
                         "FOREIGN KEY(cycle_id) REFERENCES cycle(id)"
                         ") "};

  conn.initializeTables(queries);

  // Close the database (optional, depending on your needs)
  db.close();
}

void WarehouseTab::onBrowseButtonClicked() {
  QString file_path = QFileDialog::getOpenFileName(
      this, "Select SQLite Database File", "",
      "SQLite Database (*.db *.sqlite *.sqlite3);;All Files (*)");
  if (!file_path.isEmpty()) {
    db_path_input_->setText(file_path);
  }
}

void WarehouseTab::onConnectButtonClicked() {
  QString db_path = db_path_input_->text();
  if (db_path.isEmpty()) {
    QMessageBox::warning(this, "Error", "Please provide a database file path.");
    return;
  }

  db.setDatabaseName(db_path);

  if (db.open()) {
    status_label_->setText("Status: Connected");
    connect_button_->setEnabled(false);
    disconnect_button_->setEnabled(true);
    db_path_input_->setEnabled(false); // Disable input while connected
    browse_button_->setEnabled(false); // Disable browse while connected
  } else {
    status_label_->setText("Status: Connection Failed");
    QMessageBox::critical(this, "Error",
                          "Failed to connect to database: " +
                              db.lastError().text());
  }
}

void WarehouseTab::onDisconnectButtonClicked() {
  // db.close();
  status_label_->setText("Status: Not Connected");
  connect_button_->setEnabled(true);
  disconnect_button_->setEnabled(false);
  db_path_input_->setEnabled(true);
  browse_button_->setEnabled(true);
}
