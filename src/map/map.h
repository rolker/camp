#ifndef MAP_MAP_H
#define MAP_MAP_H

#include <QAbstractItemModel>

#include "../map_view/map_view.h"

class QGraphicsScene;
class QMenu;

namespace map
{
  class MapItem;
  class TopLevelItem;
  class LayerList;

// Implements a model to use with Model/View widgets
// and provides a QGraphicsScene for viewing
// in a QGraphicsView. 
class Map: public QAbstractItemModel
{
  Q_OBJECT
public:
  explicit Map(QObject *parent = 0);
  ~Map() = default;

  // QAbstractItemModel overrides

  Qt::ItemFlags flags(const QModelIndex & index) const override;
  QVariant data(const QModelIndex & index, int role) const override;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override;
  int rowCount(const QModelIndex & parent) const override;
  int columnCount(const QModelIndex & parent) const override;
  QModelIndex index(int row, int column, const QModelIndex & parent) const override;
  QModelIndex parent(const QModelIndex & child) const override;

  bool setData(const QModelIndex &index, const QVariant &value, int role=Qt::EditRole) override;

  Qt::DropActions supportedDropActions() const override;

  QStringList mimeTypes() const override;
  QMimeData *mimeData(const QModelIndexList &indexes) const override;
  bool canDropMimeData(const QMimeData* data, Qt::DropAction action, int row, int col, const QModelIndex& parent) const override;
  bool dropMimeData(const QMimeData * data, Qt::DropAction action, int row, int col, const QModelIndex& parent) override;

  // Returns a pointer to the associated QGraphicsScene object
  // or nullptr if it can't be found.
  QGraphicsScene* scene() const;

  // Called by map items to indicate that data relevant to the tree view has changed.
  void updateDisplay(const MapItem* map_item, const QVector<int> &roles = QVector<int>());

  // Notifies the view of the change then sets the child's parent.
  void setMapItemParent(MapItem* child_item, MapItem* parent_item);

  // Sets context menu items for given index
  void contextMenuFor(QMenu* menu, const QModelIndex& index);

  LayerList* topLevelLayers() const;
signals:
  void viewportChanged(MapView::Viewport viewport);

private:
  QModelIndex index(const MapItem* map_item) const;

  MapItem* top_level_items_;
};

} // namespace map

#endif
