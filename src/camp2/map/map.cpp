#include "map.h"
#include <QGraphicsScene>
#include "../tools/tools_manager.h"
#include "layer_list.h"
#include "../background/background_manager.h"
#include "../ros/node.h"
#include "map_item_mime_data.h"
#include <QMenu>
#include "layer.h"
#include <QApplication>

#include <QDebug>

namespace camp
{
namespace map
{

Map::Map(QObject *parent):
  QAbstractItemModel(parent)
{
  auto scene = new QGraphicsScene(this);
  top_level_items_ = new MapItem("CAMP");
  scene->addItem(top_level_items_);

  new LayerList(top_level_items_);

  auto tools_manager = new tools::ToolsManager(top_level_items_);
  auto background_manager = new background::BackgroundManager(tools_manager);
  background_manager->createDefaultLayers();

  auto ros_node = new camp::ros::Node(tools_manager);
  connect(ros_node, &camp::ros::Node::shuttingDownRos, QCoreApplication::instance(), &QCoreApplication::quit);
}


QGraphicsScene* Map::scene() const
{
  for(auto child: children())
  {
    auto scene = qobject_cast<QGraphicsScene*>(child);
    if(scene)
      return scene;
  }
  return nullptr;
}


Qt::ItemFlags Map::flags(const QModelIndex & index) const
{
  Qt::ItemFlags f = QAbstractItemModel::flags(index);
  auto map_item = reinterpret_cast<MapItem*>(index.internalPointer());
  if(map_item)
    map_item->updateFlags(f);
  return f;
}


QVariant Map::data(const QModelIndex & index, int role) const
{
  QVariant data;
  auto map_item = reinterpret_cast<MapItem*>(index.internalPointer());
  if(map_item)
    switch(role)
    {
      case Qt::DisplayRole:
      {
        QString display = map_item->objectName();
        auto status = map_item->status();
        if(!status.isEmpty())
          display += " "+status;
        data.setValue(display);
        break;
      }
      case Qt::EditRole:
        data.setValue(map_item->objectName());
        break;
      case Qt::CheckStateRole:
        if(flags(index)&Qt::ItemIsUserCheckable)
        {
          if(map_item->isVisible())
            data.setValue(Qt::Checked);
          else
            data.setValue(Qt::Unchecked);
        }
        break;
    }

  return data;
}

bool Map::setData(const QModelIndex &index, const QVariant &value, int role)
{
  auto map_item = reinterpret_cast<MapItem*>(index.internalPointer());
  if(map_item)
    switch(role)
    {
      case Qt::EditRole:
        map_item->setObjectName(value.value<QString>());
        emit dataChanged(index, index, QVector<int>(1,role));
        return true;
      case Qt::CheckStateRole:
        map_item->setVisible(value.value<Qt::CheckState>());
        emit dataChanged(index, index, QVector<int>(1,role));
        return true;
    }
  return false;
}

QVariant Map::headerData(int section, Qt::Orientation orientation, int role) const
{
  QVariant data;
  if(orientation == Qt::Horizontal && role == Qt::DisplayRole && section == 0)
    data.setValue(QString("Name"));
  return data;
}

int Map::rowCount(const QModelIndex & parent) const
{
  if(parent.isValid())
  {
    if(parent.column() == 0)
    {
      auto map_item = reinterpret_cast<MapItem*>(parent.internalPointer());
      if(map_item)
        return map_item->childMapItems().size();
    }
    return 0;
  }
  return top_level_items_->childMapItems().size();
}


int Map::columnCount(const QModelIndex & parent) const
{
  if(!parent.isValid() || parent.column() == 0)
    return 1;
  return 0;
}


QModelIndex Map::index(int row, int column, const QModelIndex& parent) const
{
  MapItem* parent_map_item = reinterpret_cast<MapItem*>(parent.internalPointer());
  if(!parent_map_item)
    parent_map_item =top_level_items_;

  if(!parent.isValid() || parent.column() == 0)
  {
    if(row >= 0 && column == 0)
    {
      auto map_items = parent_map_item->childMapItems();
      if(row < map_items.size())
      {
        // Items are stored in order they are rendered, but we
        // want to show items which are drawn last at the top of the list.
        // For example, a ship track should be drawn over a background map so will be later in the list than the background map.
        int list_row = map_items.size()-1-row;
        return createIndex(row, column, map_items[list_row]);
      }
    }
  }
  return QModelIndex();
}

QModelIndex Map::index(const MapItem* map_item) const
{
  if(map_item != nullptr && map_item != top_level_items_)
  {
    auto parent_item = map_item->parentMapItem();
    if(parent_item)
    {
      auto parent_index = index(parent_item);
      if(!parent_index.isValid() && parent_item != top_level_items_)
      {
        // parent not in model
        return QModelIndex();
      }
      auto siblings = parent_item->childConstMapItems();
      // reverse list index, see index(int, int, const QModelIndex&) for details
      int list_row = siblings.size()-1-siblings.indexOf(map_item);
      auto ret = index(list_row, 0, index(parent_item));
      assert(map_item == ret.internalPointer());
      return ret;
    }
  }
  return QModelIndex();
}


QModelIndex Map::parent(const QModelIndex & child) const
{
  MapItem* child_item = reinterpret_cast<MapItem*>(child.internalPointer());
  if(child_item)
  {
    MapItem* parent_item = child_item->parentMapItem();
    assert(((void*)(parent_item) != this));
    if(parent_item)
    {
      auto grandparent_item = parent_item->parentMapItem();
      assert(((void*)(grandparent_item) != this));
      if(grandparent_item)
      {
        // Reverse row index number. See Map::index for details.
        auto parent_siblings = grandparent_item->childMapItems();
        int parent_row = parent_siblings.size()-1-parent_siblings.indexOf(parent_item);

        return createIndex(parent_row, 0, parent_item);
      }
    }
  }
  return QModelIndex();
}

Qt::DropActions Map::supportedDropActions() const
{
  return Qt::MoveAction;
}

QStringList Map::mimeTypes() const
{
  QStringList types;
  types << MapItem::MimeType;
  return types;
}

QMimeData * Map::mimeData(const QModelIndexList &indexes) const
{
  if(!indexes.empty())
  {
    auto map_item = reinterpret_cast<MapItem*>(indexes.front().internalPointer());
    if(map_item)
    {
      MapItemMimeData *mimeData = new MapItemMimeData;
      mimeData->setMapItem(map_item);
      return mimeData;
    }
  }
  return nullptr;
}

bool Map::canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int col, const QModelIndex& parent) const
{
  auto parent_item = reinterpret_cast<MapItem*>(parent.internalPointer());
  if(parent_item)
    return parent_item->canDropMimeData(data, action, row, col);
  return false;
}

bool Map::dropMimeData(const QMimeData * data, Qt::DropAction action, int row, int col, const QModelIndex& parent)
{
  auto parent_item = reinterpret_cast<MapItem*>(parent.internalPointer());
  if(parent_item)
    if(action == Qt::MoveAction && col == 0)
    {
      auto map_item_data = qobject_cast<const MapItemMimeData*>(data);
      if(map_item_data)
      {
        auto map_item = map_item_data->mapItem();
        if(map_item)
        {
          setMapItemParent(map_item, parent_item, row);
          return true;
        }
      }
    }
  return false;
}

void Map::updateDisplay(const MapItem* map_item, const QVector<int> &roles)
{
  auto item_index = index(map_item);
  emit dataChanged(item_index, item_index, roles);
}


void Map::setMapItemParent(MapItem* child_item, MapItem* parent_item, int row)
{
  if(!child_item)
    return;

  // Check if already in the model
  auto existing_index = index(child_item);
  if(existing_index.isValid())
  {
    auto old_parent_item = child_item->parentMapItem();
    if(old_parent_item == parent_item)
    {
      if(existing_index.row() == row)
        return; // no change
      // if we are moving down in the same list, account for our old spot.
      if(existing_index.row() < row)
        row -= 1;
    }

    beginRemoveRows(existing_index.parent(), existing_index.row(), existing_index.row());
    child_item->setParentItem(nullptr);
    endRemoveRows();
  }
 
  if(parent_item)
  {
    auto parent_index = index(parent_item);
    beginInsertRows(parent_index, row, row);
    child_item->setParentItem(parent_item);
    if(row > 0)
    {
      auto siblings = parent_item->childMapItems();
      auto stack_before_target = siblings.rbegin();
      for(int i = 0; i < row && stack_before_target+1 != siblings.rend(); i++)
        ++stack_before_target;
      if(child_item != *stack_before_target)
        child_item->stackBefore(*stack_before_target);
    }
    endInsertRows();
  }
  else
  {
    child_item->setParentItem(nullptr);
  }
}

void Map::contextMenuFor(QMenu* menu, const QModelIndex& index)
{
  auto map_item = reinterpret_cast<MapItem*>(index.internalPointer());
  if(map_item)
    map_item->contextMenu(menu);
}

LayerList* Map::topLevelLayers() const
{
  for(auto item: top_level_items_->childMapItems())
  {
    auto layers = qgraphicsitem_cast<LayerList*>(item);
    if(layers)
      return layers;
  }
  return nullptr;
}


} // namespace map
} // namespace camp
