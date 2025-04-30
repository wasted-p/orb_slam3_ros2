#ifndef WAREHOUSE_TAB_H
#define WAREHOUSE_TAB_H

#include <QPushButton>
#include <QSqlQuery>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlError>

#include <QItemDelegate>
#include <qlabel.h>
#include <qpushbutton.h>

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
  QLabel *status_label;
  QPushButton *connect_button;
  QPushButton *disconnect_button;
  QPushButton *db_path_input;
  QPushButton *browse_button;

  mutable QSqlDatabase db;
};

#endif // WAREHOUSE_TAB_H
