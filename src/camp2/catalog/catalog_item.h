#ifndef CATALOG_CATALOG_ITEM_H
#define CATALOG_CATALOG_ITEM_H

#include <QString>
#include <memory>
#include <vector>

namespace camp
{
namespace catalog
{

/// [camp#104] A node in a generic browse catalog. A node is either a *group*
/// (has children, not selectable for "add to map") or a *selectable leaf*
/// carrying an opaque payload — a source id plus a string key that the owning
/// CatalogSource understands (for GGGS the key is a tile-set directory path).
/// CatalogItem knows nothing about GGGS, raster, or layers; it is the
/// source-agnostic shape the CatalogModel presents.
class CatalogItem
{
public:
  /// Group node (not selectable).
  explicit CatalogItem(const QString& name):
    name_(name)
  {
  }

  /// Selectable leaf node carrying the source id + opaque key.
  CatalogItem(const QString& name, const QString& source_id, const QString& key):
    name_(name), source_id_(source_id), key_(key), leaf_(true)
  {
  }

  const QString& name() const { return name_; }
  bool isLeaf() const { return leaf_; }
  const QString& sourceId() const { return source_id_; }
  const QString& key() const { return key_; }

  /// Append @p child, taking ownership and setting its parent. Returns a raw
  /// pointer to the inserted child (ownership stays with this node).
  CatalogItem* addChild(std::unique_ptr<CatalogItem> child)
  {
    child->parent_ = this;
    children_.push_back(std::move(child));
    return children_.back().get();
  }

  int childCount() const { return static_cast<int>(children_.size()); }
  CatalogItem* child(int row) const
  {
    if(row < 0 || row >= childCount())
      return nullptr;
    return children_[static_cast<size_t>(row)].get();
  }

  CatalogItem* parent() const { return parent_; }

  /// Index of this node within its parent's child list (0 for a parentless
  /// root or an orphan).
  int row() const
  {
    if(!parent_)
      return 0;
    const auto& siblings = parent_->children_;
    for(size_t i = 0; i < siblings.size(); ++i)
      if(siblings[i].get() == this)
        return static_cast<int>(i);
    return 0;
  }

private:
  QString name_;
  QString source_id_;
  QString key_;
  bool leaf_ = false;
  CatalogItem* parent_ = nullptr;
  std::vector<std::unique_ptr<CatalogItem>> children_;
};

}  // namespace catalog
}  // namespace camp

#endif
