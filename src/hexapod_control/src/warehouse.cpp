#include "hexapod_control/warehouse.hpp"
#include <iostream>

// Initialize static member
std::unique_ptr<WarehouseConnection> WarehouseConnection::instance_ = nullptr;

WarehouseConnection &WarehouseConnection::getInstance(const QString &dbPath) {
  if (!instance_) {
    instance_ =
        std::unique_ptr<WarehouseConnection>(new WarehouseConnection(dbPath));
  }
  return *instance_;
}

WarehouseConnection::WarehouseConnection(const QString &dbPath) {
  if (!initialize(dbPath)) {
    qCritical() << "Failed to initialize database connection to" << dbPath;
  }
}

WarehouseConnection::~WarehouseConnection() {
  if (db_.isOpen()) {
    db_.close();
    qDebug() << "Database connection closed";
  }
}

bool WarehouseConnection::initialize(const QString &dbPath) {
  db_ = QSqlDatabase::addDatabase("QSQLITE");
  db_.setDatabaseName(dbPath);

  if (!db_.open()) {
    qCritical() << "Error opening database:" << db_.lastError().text();
    return false;
  }

  qDebug() << "Database connection established successfully";
  return true;
}

bool WarehouseConnection::initializeTables(
    const QStringList &createTableQueries) {
  for (const QString &query : createTableQueries) {
    QSqlQuery q;
    if (!q.exec(query)) {
      qCritical() << "Failed to create table:" << q.lastError().text();
      qCritical() << "Query was:" << query;
      return false;
    }
  }
  return true;
}

bool WarehouseConnection::insert(const QString &tableName,
                                 const QMap<QString, QVariant> &values) {
  if (values.isEmpty()) {
    qWarning() << "No values provided for insert operation";
    return false;
  }

  QStringList columns;
  QStringList placeholders;
  QVariantList valuesList;

  for (auto it = values.begin(); it != values.end(); ++it) {
    columns.append(it.key());
    placeholders.append(":" + it.key()); // Using named placeholders
    valuesList.append(it.value());
  }

  QString queryStr = QString("INSERT INTO %1 (%2) VALUES (%3)")
                         .arg(tableName)
                         .arg(columns.join(", "))
                         .arg(placeholders.join(", "));

  QSqlQuery query;
  query.prepare(queryStr);

  for (const QVariant &value : valuesList) {
    query.addBindValue(value);
  }

  if (!query.exec()) {
    qCritical() << "Insert failed:" << query.lastError().text();
    qCritical() << "Query was:" << queryStr;
    return false;
  }

  qDebug() << "Record inserted successfully into" << tableName;
  return true;
}

bool WarehouseConnection::update(const QString &tableName,
                                 const QMap<QString, QVariant> &values,
                                 const QString &whereCondition) {
  if (values.isEmpty()) {
    qWarning() << "No values provided for update operation";
    return false;
  }

  QStringList setStatements;
  QVariantList valuesList;

  for (auto it = values.begin(); it != values.end(); ++it) {
    setStatements.append(it.key() + " = ?");
    valuesList.append(it.value());
  }

  QString queryStr =
      QString("UPDATE %1 SET %2").arg(tableName).arg(setStatements.join(", "));

  if (!whereCondition.isEmpty()) {
    queryStr += " WHERE " + whereCondition;
  }

  QSqlQuery query;
  query.prepare(queryStr);

  for (const QVariant &value : valuesList) {
    query.addBindValue(value);
  }

  if (!query.exec()) {
    qCritical() << "Update failed:" << query.lastError().text();
    qCritical() << "Query was:" << queryStr;
    return false;
  }

  qDebug() << "Record(s) updated successfully in" << tableName;
  return true;
}

bool WarehouseConnection::remove(const QString &tableName,
                                 const QString &whereCondition) {
  QString queryStr = QString("DELETE FROM %1").arg(tableName);

  if (!whereCondition.isEmpty()) {
    queryStr += " WHERE " + whereCondition;
  } else {
    qWarning() << "Performing DELETE without WHERE condition on table"
               << tableName;
  }

  QSqlQuery query;
  if (!query.exec(queryStr)) {
    qCritical() << "Delete failed:" << query.lastError().text();
    qCritical() << "Query was:" << queryStr;
    return false;
  }

  int rowsAffected = query.numRowsAffected();
  qDebug() << rowsAffected << "record(s) deleted successfully from"
           << tableName;
  return true;
}

QSqlQuery WarehouseConnection::executeQuery(const QString &queryStr) {
  QSqlQuery query;
  if (!query.exec(queryStr)) {
    qCritical() << "Query execution failed:" << query.lastError().text();
    qCritical() << "Query was:" << queryStr;
  }
  return query;
}
