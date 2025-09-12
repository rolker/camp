#ifndef MAP_MAP_ITEM_H
#define MAP_MAP_ITEM_H

#include "item_types.h"

class QMimeData;
namespace camp
{
namespace map
{

class Map;

/// Base class for map components.
/// MapItems are QGraphicsObjects so they can be displayed
/// in a MapView and also implement the methods needed
/// by Map for use in a MapTreeView.
/// By inheriting from QGraphicsObject, MapItem is also 
/// a QObject and can use signals and slots.
class MapItem: public QGraphicsObject
{
  Q_OBJECT
public:
  MapItem(MapItem* parent_item, const QString& object_name);
  virtual ~MapItem() = default;

  enum { Type = MapItemType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  // QGraphicsItem overrides
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  /// Makes sure the Map is updated as needed.
  virtual void setParentMapItem(MapItem* parent);

  /// Returns a pointer to the parent item if it is a MapItem.
  /// If the parent is not a MapItem or is null, return nullptr.
  MapItem* parentMapItem() const;

  template<typename T>
  T* parentOfType() const
  {
    auto item = parentMapItem();
    while(item)
    {
      auto cast_item = qgraphicsitem_cast<T*>(item);
      if(cast_item)
        return cast_item;
      item = item->parentMapItem();
    }
    return nullptr;
  }

  template<typename T>
  T* firstChildOfType(bool recursive = false) const
  {
    for(auto item: childMapItems())
    {
      auto cast_item = qgraphicsitem_cast<T*>(item);
      if(cast_item)
        return cast_item;
    }
    if(recursive)
    {
      for(auto item: childMapItems())
      {
        auto descendant_item = item->firstChildOfType<T>(true);
        if(descendant_item)
          return descendant_item;
      }
    }
    return nullptr;
  }

  /// Returns true if this item is an ancestor of the given item.
  bool isAncestorOf(const MapItem* item) const
  {
    while(item)
    {
      if(item->parentMapItem() == this)
        return true;
      item = item->parentMapItem();
    }
    return false;
  }

  MapItem* commonAncestor(MapItem* other) const
  {
    if(other == nullptr)
      return nullptr;
    auto ancestor = parentMapItem();
    while(ancestor)
    {
      if(ancestor->isAncestorOf(other))
        return ancestor;
      ancestor = ancestor->parentMapItem();
    }
    return nullptr;
  }

  /// Return the child items that are MapItems.
  virtual QList<MapItem*> childMapItems(bool recursive = false) const;
  QList<const MapItem*> childConstMapItems(bool recursive = false) const;

  static constexpr char MimeType[] = "application/camp.map_item.pointer";

  virtual bool canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int col) const;

  // Sets the object name and notifies the model of the change.
  void setObjectName(const QString& name);

  // Returns the status text to be displayed in the tree view.
  const QString& status() const;

public slots:
  void setOpacity(qreal opacity);

protected:
  // Allows item to modify Model/View flags.
  virtual void updateFlags(Qt::ItemFlags& flags) const;
  
  // Sets the status text to be displayed in tree view.
  void setStatus(const QString& status);

  // Returns the Map object this belongs to, or nullptr if not found.
  Map * parentMap() const;

  // called when a context menu is requested.
  virtual void contextMenu(QMenu* menu);

  virtual void readSettings();
  virtual void writeSettings();

private:
  // Make sure Map can create a top level item without a parent.
  friend class Map;
  MapItem(const QString& object_name);

private slots:
  // called once the MapItem and derived constructors are completed.
  void itemConstructed();
  void applicationQuitting();

private:
  // Status to be displayed along object name in tree view.
  QString status_;
};

} // namespace map
} // namespace camp

#endif
