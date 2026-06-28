#ifndef CAMP_ROS_NAMES_MANAGER_H
#define CAMP_ROS_NAMES_MANAGER_H

#include "../tools/map_tool.h"
#include "ros_common.h"
#include <QQueue>

namespace camp
{
namespace ros
{

class Name;

/// Manages a list of ROS entity names such as topics or services.
class NamesManager: public tools::MapTool
{
  Q_OBJECT
public:
  NamesManager(MapTool* parent, const QString& object_name);

  enum { Type = map::RosNamesManagerType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }


  /// Sets an optional filter to limit displayed names to those matching one of the types in the list.
  /// If empty, all names are shown.
  void setTypeFilter(const std::vector<std::string>& type_filter);

  void clearTypeFilter();

  std::map<std::string, std::vector<std::string>> namesAndTypes() const;

signals:
  void namesUpdated();

public slots:
  void updateName(QString name, QStringList types);
  void addName(QString name);
  void removeName(QString name);

protected:

  /// Adds a new name to the manager.
  Name* add(const std::string& full_name);

  /// Removes a name from the manager.
  void remove(const std::string& full_name);

private slots:
  void processPendingUpdates();

private:
  Name* root_ = nullptr;

  QQueue<std::pair< QString, QStringList>> pending_updates_;

  /// Optional filter to limit displayed names to those matching one of the types in the list.
  /// If empty, all names are shown.
  std::vector<std::string> type_filter_;

};

class TopicsManager: public NamesManager
{
  Q_OBJECT
public:
  TopicsManager(MapTool* parent, const QString& object_name = "Topics");
};

class ServicesManager: public NamesManager
{
  Q_OBJECT
public:
  ServicesManager(MapTool* parent, const QString& object_name = "Services");
};

class NodesManager: public NamesManager
{
  Q_OBJECT
public:
  NodesManager(MapTool* parent, const QString& object_name = "Nodes");
};

} // namespace ros
} // namespace camp

#endif
