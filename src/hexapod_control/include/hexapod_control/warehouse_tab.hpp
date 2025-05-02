#ifndef WAREHOUSE_TAB_H
#define WAREHOUSE_TAB_H

#include <QPushButton>
#include <QSqlQuery>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>

#include <QDebug>
#include <QFileDialog>
#include <QHeaderView>
#include <QItemDelegate>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSqlDatabase>
#include <QSqlError>
#include <QtGlobal>

class WarehouseTab : public QWidget {
  Q_OBJECT
public:
  explicit WarehouseTab(QWidget *parent = nullptr);

private slots:
  void onCreateButtonClicked();

  void onBrowseButtonClicked();

  void onConnectButtonClicked();

  void onDisconnectButtonClicked();

private:
  QLabel *status_label_;
  QPushButton *connect_button_;
  QPushButton *create_button_;
  QPushButton *disconnect_button_;
  QLineEdit *db_path_input_;
  QPushButton *browse_button_;

  mutable QSqlDatabase db;
};

#endif // WAREHOUSE_TAB_H
