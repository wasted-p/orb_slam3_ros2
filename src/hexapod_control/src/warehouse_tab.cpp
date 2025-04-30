#include "hexapod_control/warehouse_tab.hpp"
#include <QDebug>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QSqlDatabase>
#include <qfiledialog.h>
#include <qmessagebox.h>

WarehouseTab::WarehouseTab(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *layout = new QVBoxLayout;
  setLayout(layout);

  // Title label
  QLabel *displays_label = new QLabel("View Warehouse Connection");
  layout->addWidget(displays_label);

  // Database file path input
  QLabel *db_path_label = new QLabel("Database File Path:");
  QLineEdit *db_path_input = new QLineEdit;
  db_path_input->setPlaceholderText(
      "Enter or select SQLite database file path");
  layout->addWidget(db_path_label);
  layout->addWidget(db_path_input);

  // Browse button for selecting database file
  QPushButton *browse_button = new QPushButton("Browse...");
  layout->addWidget(browse_button);

  // Connect and Disconnect buttons
  QPushButton *create_button = new QPushButton("Create");
  QPushButton *connect_button = new QPushButton("Connect");
  QPushButton *disconnect_button = new QPushButton("Disconnect");
  disconnect_button->setEnabled(false); // Initially disabled until connected
  QHBoxLayout *button_layout = new QHBoxLayout;
  button_layout->addWidget(connect_button);
  button_layout->addWidget(create_button);
  button_layout->addWidget(disconnect_button);
  layout->addLayout(button_layout);

  // Status label for connection feedback
  status_label = new QLabel("Status: Not Connected");
  layout->addWidget(status_label);

  // Spacer to push content to the top
  layout->addStretch();

  // Setup layout
  // QVBoxLayout *layout = new QVBoxLayout(this);
  // QHBoxLayout *button_layout = new QHBoxLayout;
  // button_layout->addWidget(create_button_);
  // button_layout->addWidget(delete_button_);
  // button_layout->addWidget(save_button_);
  // button_layout->addWidget(move_up_button_);
  // button_layout->addWidget(move_down_button_);
  // layout->addWidget(table_);
  // layout->addLayout(button_layout);
  // setLayout(layout);

  // Setup table

  // Connect signals
  // connect(create_button_, &QPushButton::clicked, this,
  //         &WarehouseTab::onCreateButtonClicked);
  // connect(delete_button_, &QPushButton::clicked, this,
  //         &WarehouseTab::onDeleteButtonClicked);
  // connect(save_button_, &QPushButton::clicked, this,
  //         &WarehouseTab::onSaveButtonClicked);
  // connect(move_up_button_, &QPushButton::clicked, this,
  //         &WarehouseTab::onMoveUpButtonClicked);
  // connect(move_down_button_, &QPushButton::clicked, this,
  //         &WarehouseTab::onMoveDownButtonClicked);
  //
  // // Add a sample row
  // addRow(1.0f, "Pose1", 0.0f);
}

void WarehouseTab::onCreateButtonClicked() {
  // Specify the database path (replace with your desired path)
  QString dbPath = "/home/theycallmemuzz/Code/hexapod-ros/warehouse.db";

  // Create and configure the SQLite database
  QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", "hexapod_connection");
  db.setDatabaseName(dbPath);

  // Open the database (this creates the file if it doesn't exist)
  if (!db.open()) {
    qDebug() << "Failed to create/open database:" << db.lastError().text();
    return;
  }

  qDebug() << "Database created/opened successfully at" << dbPath;

  // Optional: Create a sample table (e.g., for hexapod data)
  QSqlQuery query(db);

  bool success = query.exec("CREATE TABLE IF NOT EXISTS hexapod_data ("
                            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
                            "timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,"
                            " data TEXT) ");

  if (!success) {
    qDebug() << "Failed to create table:" << query.lastError().text();
  } else {
    qDebug() << "Table created successfully";
  }

  // Close the database (optional, depending on your needs)
  db.close();
}

void WarehouseTab::onBrowseButtonClicked() {
  QString file_path = QFileDialog::getOpenFileName(
      this, "Select SQLite Database File", "",
      "SQLite Database (*.db *.sqlite *.sqlite3);;All Files (*)");
  if (!file_path.isEmpty()) {
    db_path_input->setText(file_path);
  }
}

void WarehouseTab::onConnectButtonClicked() {
  QString db_path = db_path_input->text();
  if (db_path.isEmpty()) {
    QMessageBox::warning(this, "Error", "Please provide a database file path.");
    return;
  }

  db.setDatabaseName(db_path);

  if (db.open()) {
    status_label->setText("Status: Connected");
    connect_button->setEnabled(false);
    disconnect_button->setEnabled(true);
    db_path_input->setEnabled(false); // Disable input while connected
    browse_button->setEnabled(false); // Disable browse while connected
  } else {
    status_label->setText("Status: Connection Failed");
    QMessageBox::critical(this, "Error",
                          "Failed to connect to database: " +
                              db.lastError().text());
  }
}

void WarehouseTab::onDisconnectButtonClicked() {
  // db.close();
  status_label->setText("Status: Not Connected");
  connect_button->setEnabled(true);
  disconnect_button->setEnabled(false);
  db_path_input->setEnabled(true);
  browse_button->setEnabled(true);
}
