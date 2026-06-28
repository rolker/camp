#ifndef CATALOG_CATALOG_MODEL_H
#define CATALOG_CATALOG_MODEL_H

#include <QAbstractItemModel>
#include <memory>

#include "catalog_item.h"

namespace camp
{
namespace catalog
{

/// [camp#104] Generic hierarchical model over a CatalogItem tree, presented in
/// the CatalogBrowser's tree view. It is source-agnostic: it holds an invisible
/// root whose children are the top-level catalog nodes added by discovery (one
/// per browsed source root). Groups are display-only; leaves carry the
/// source-id + key payload a CatalogSource uses to instantiate a layer.
class CatalogModel: public QAbstractItemModel
{
  Q_OBJECT
public:
  explicit CatalogModel(QObject* parent = nullptr);

  // QAbstractItemModel overrides
  QModelIndex index(int row, int column, const QModelIndex& parent) const override;
  QModelIndex parent(const QModelIndex& child) const override;
  int rowCount(const QModelIndex& parent) const override;
  int columnCount(const QModelIndex& parent) const override;
  QVariant data(const QModelIndex& index, int role) const override;
  Qt::ItemFlags flags(const QModelIndex& index) const override;

  /// Append a discovered top-level node (e.g. a browsed store root), taking
  /// ownership. Wrapped in begin/endInsertRows so attached views update.
  void addTopLevel(std::unique_ptr<CatalogItem> item);

  /// Remove the top-level node at @p row (e.g. an operator-removed browsed
  /// root). Wrapped in begin/endRemoveRows so attached views update. No-op for
  /// an out-of-range row.
  void removeTopLevel(int row);

  /// Remove all catalog nodes (e.g. before re-browsing).
  void clear();

  /// The CatalogItem behind @p index, or nullptr for an invalid index.
  const CatalogItem* itemForIndex(const QModelIndex& index) const;

private:
  CatalogItem* nodeForIndex(const QModelIndex& index) const;

  std::unique_ptr<CatalogItem> root_;
};

}  // namespace catalog
}  // namespace camp

#endif
