#ifndef WAREHOUSE_HPP
#define WAREHOUSE_HPP

#include <QDebug>
#include <QMap>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QString>
#include <QVariant>
#include <memory>

/**
 * @brief The WarehouseConnection class provides a singleton interface for
 * SQLite database operations
 */
class WarehouseConnection {
public:
  /**
   * @brief getInstance returns the singleton instance of WarehouseConnection
   * @param dbPath path to the SQLite database file
   * @return reference to the singleton instance
   */
  static WarehouseConnection &
  getInstance(const QString &dbPath = "database.db");

  /**
   * @brief Destructor
   */
  ~WarehouseConnection();

  /**
   * @brief insert record into specified table
   * @param tableName name of the table
   * @param values map of column names to values
   * @return true if successful, false otherwise
   */
  bool insert(const QString &tableName, const QMap<QString, QVariant> &values);

  /**
   * @brief update record in specified table
   * @param tableName name of the table
   * @param values map of column names to values to update
   * @param whereCondition WHERE clause condition
   * @return true if successful, false otherwise
   */
  bool update(const QString &tableName, const QMap<QString, QVariant> &values,
              const QString &whereCondition);

  /**
   * @brief delete record from specified table
   * @param tableName name of the table
   * @param whereCondition WHERE clause condition
   * @return true if successful, false otherwise
   */
  bool remove(const QString &tableName, const QString &whereCondition);

  /**
   * @brief executes a custom query
   * @param queryStr the SQL query string
   * @return QSqlQuery object with the results
   */
  QSqlQuery executeQuery(const QString &queryStr);

  /**
   * @brief initializes database tables
   * @param createTableQueries list of CREATE TABLE queries
   * @return true if successful, false otherwise
   */
  bool initializeTables(const QStringList &createTableQueries);

private:
  /**
   * @brief Private constructor for singleton pattern
   * @param dbPath path to the SQLite database file
   */
  explicit WarehouseConnection(const QString &dbPath);

  /**
   * @brief Deleted copy constructor
   */
  WarehouseConnection(const WarehouseConnection &) = delete;

  /**
   * @brief Deleted assignment operator
   */
  WarehouseConnection &operator=(const WarehouseConnection &) = delete;

  /**
   * @brief Initialize the database connection
   * @param dbPath path to the SQLite database file
   * @return true if successful, false otherwise
   */
  bool initialize(const QString &dbPath);

  QSqlDatabase db_; ///< The database connection object
  static std::unique_ptr<WarehouseConnection> instance_; ///< Singleton instance
};

#endif // DATABASEMANAGER_HPP
