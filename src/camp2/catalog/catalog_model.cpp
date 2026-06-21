#include "catalog_model.h"

namespace camp
{
namespace catalog
{

CatalogModel::CatalogModel(QObject* parent):
  QAbstractItemModel(parent),
  root_(std::make_unique<CatalogItem>(QString()))   // invisible group root
{
}

CatalogItem* CatalogModel::nodeForIndex(const QModelIndex& index) const
{
  if(!index.isValid())
    return root_.get();
  return static_cast<CatalogItem*>(index.internalPointer());
}

const CatalogItem* CatalogModel::itemForIndex(const QModelIndex& index) const
{
  if(!index.isValid())
    return nullptr;
  return static_cast<const CatalogItem*>(index.internalPointer());
}

QModelIndex CatalogModel::index(int row, int column, const QModelIndex& parent) const
{
  if(!hasIndex(row, column, parent))
    return QModelIndex();
  CatalogItem* parent_item = nodeForIndex(parent);
  CatalogItem* child = parent_item->child(row);
  if(!child)
    return QModelIndex();
  return createIndex(row, column, child);
}

QModelIndex CatalogModel::parent(const QModelIndex& child) const
{
  if(!child.isValid())
    return QModelIndex();
  CatalogItem* child_item = nodeForIndex(child);
  CatalogItem* parent_item = child_item ? child_item->parent() : nullptr;
  if(!parent_item || parent_item == root_.get())
    return QModelIndex();
  return createIndex(parent_item->row(), 0, parent_item);
}

int CatalogModel::rowCount(const QModelIndex& parent) const
{
  if(parent.column() > 0)
    return 0;
  return nodeForIndex(parent)->childCount();
}

int CatalogModel::columnCount(const QModelIndex& parent) const
{
  Q_UNUSED(parent);
  return 1;
}

QVariant CatalogModel::data(const QModelIndex& index, int role) const
{
  if(!index.isValid() || role != Qt::DisplayRole)
    return QVariant();
  return nodeForIndex(index)->name();
}

Qt::ItemFlags CatalogModel::flags(const QModelIndex& index) const
{
  if(!index.isValid())
    return Qt::NoItemFlags;
  // Groups are display-only (expandable but not selectable for "add to map");
  // only leaves are selectable so the browser's add affordance targets a payload.
  CatalogItem* item = nodeForIndex(index);
  Qt::ItemFlags flags = Qt::ItemIsEnabled;
  if(item->isLeaf())
    flags |= Qt::ItemIsSelectable;
  return flags;
}

void CatalogModel::addTopLevel(std::unique_ptr<CatalogItem> item)
{
  const int row = root_->childCount();
  beginInsertRows(QModelIndex(), row, row);
  root_->addChild(std::move(item));
  endInsertRows();
}

void CatalogModel::clear()
{
  if(root_->childCount() == 0)
    return;
  beginResetModel();
  root_ = std::make_unique<CatalogItem>(QString());
  endResetModel();
}

}  // namespace catalog
}  // namespace camp
