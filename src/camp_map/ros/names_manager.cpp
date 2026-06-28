#include "names_manager.h"
#include "name.h"
#include "../map/map.h"
#include "node.h"
#include "graph_thread.h"
#include <QTimer>

namespace camp
{
namespace ros
{

NamesManager::NamesManager(MapTool* parent, const QString& object_name):
  tools::MapTool(parent, object_name)
{
  root_ = new Name(this);
}

TopicsManager::TopicsManager(MapTool* parent, const QString& object_name):
  NamesManager(parent, object_name)
{
  auto node = parentOfType<Node>();
  if(node)
  {
    connect(node->graphThread(), &GraphThread::updateTopic, this, &NamesManager::updateName, Qt::QueuedConnection);
  }
}

ServicesManager::ServicesManager(MapTool* parent, const QString& object_name):
  NamesManager(parent, object_name)
{
  auto node = parentOfType<Node>();
  if(node)
  {
    connect(node->graphThread(), &GraphThread::updateService, this, &NamesManager::updateName, Qt::QueuedConnection);
  }
}

NodesManager::NodesManager(MapTool* parent, const QString& object_name):
  NamesManager(parent, object_name)
{
  auto node = parentOfType<Node>();
  if(node)
  {
    connect(node->graphThread(), &GraphThread::addNode, this, &NamesManager::addName, Qt::QueuedConnection);
    connect(node->graphThread(), &GraphThread::removeNode, this, &NamesManager::removeName, Qt::QueuedConnection);
  }
}

void NamesManager::setTypeFilter(const std::vector<std::string>& type_filter)
{
  type_filter_ = type_filter;
}

void NamesManager::clearTypeFilter()
{
  type_filter_.clear();
}


Name* NamesManager::add(const std::string& name)
{
  return root_->add(name);
}

void NamesManager::remove(const std::string& name)
{
  auto name_item = root_->find(name);
  if(name_item && name_item != root_)
  {
    name_item->set_types(std::vector<std::string>());
    if(name_item->entity_names().empty())
    {
      name_item->setParentMapItem(nullptr);
      name_item->deleteLater();
    }
  }
}

void NamesManager::addName(QString name)
{
  bool trigger_updates = pending_updates_.isEmpty();
  QStringList types;
  // add empty types to indicate a valid name but without a type
  types << "";
  pending_updates_.append(std::make_pair(name, types));

  if(trigger_updates)
  {
    setStatus(QString("(updating...)"));
    QTimer::singleShot(0, this, &NamesManager::processPendingUpdates);
  }
}

void NamesManager::removeName(QString name)
{
  bool trigger_updates = pending_updates_.isEmpty();
  QStringList types;
  pending_updates_.append(std::make_pair(name, types));

  if(trigger_updates)
  {
    setStatus(QString("(updating...)"));
    QTimer::singleShot(0, this, &NamesManager::processPendingUpdates);
  }
}

void NamesManager::updateName(QString name, QStringList types)
{
  bool trigger_updates = pending_updates_.isEmpty();
  QStringList filtered_types;
  if(type_filter_.empty())
  {
    filtered_types = types;
  }
  else
  {
    for(const auto& type: types)
    {
      if(std::find(type_filter_.begin(), type_filter_.end(), type.toStdString()) != type_filter_.end())
        filtered_types.append(type);
    }
  }
  pending_updates_.append(std::make_pair(name, filtered_types));
  if(trigger_updates)
  {
    setStatus(QString("(updating...)"));
    QTimer::singleShot(0, this, &NamesManager::processPendingUpdates);
  }

}

void NamesManager::processPendingUpdates()
{
  auto start_time = std::chrono::steady_clock::now();
  auto deadline = start_time + std::chrono::milliseconds(100);
  while(!pending_updates_.isEmpty())
  {
    auto update = pending_updates_.takeFirst();
    if(update.second.isEmpty())
    {
      // remove the name
      auto name_item = root_->find(update.first.toStdString());
      if(name_item)
      {
        name_item->set_types(std::vector<std::string>());
        if(name_item->entity_names().empty() && name_item != root_)
        {
          name_item->setParentMapItem(nullptr);
          name_item->deleteLater();
        }
      }
    }
    else
    {
      auto name_item = add(update.first.toStdString());
      std::vector<std::string> type_vector;
      for(const auto& type: update.second)
        type_vector.push_back(type.toStdString());
      name_item->set_types(type_vector);
    }

    auto empty_namespaces = root_->empty_namespaces();
    for(auto name_item: empty_namespaces)
    {
      if(name_item == root_)
        continue;
      name_item->setParentMapItem(nullptr);
      name_item->deleteLater();
    }
    if(std::chrono::steady_clock::now() > deadline)
    {
      // reschedule to avoid blocking the main thread too long
      QTimer::singleShot(0, this, &NamesManager::processPendingUpdates);
      return;
    }
  }
  setStatus(QString());
  emit namesUpdated();
}


std::map<std::string, std::vector<std::string>> NamesManager::namesAndTypes() const
{
  std::map<std::string, std::vector<std::string>> names_and_types;
  auto entity_names = root_->entity_names();
  for(auto name_item: entity_names)
  {
    names_and_types[name_item->full_name()] = name_item->types();
  }
  return names_and_types;
}

} // namespace ros
} // namespace camp
