#ifndef MAP_TREE_VIEW_MAP_ITEM_DELEGATE_H
#define MAP_TREE_VIEW_MAP_ITEM_DELEGATE_H

#include <QStyledItemDelegate>

namespace map_tree_view
{

class MapItemDelegate: public QStyledItemDelegate
{
public:
  MapItemDelegate(QObject *parent = nullptr);

  QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
  void setEditorData(QWidget *editor, const QModelIndex &index) const override;
  void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
  void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};

} // namespace map_tree_view

#endif
