#ifndef MAP_MAP_ITEM_DELEGATE_H
#define MAP_MAP_ITEM_DELEGATE_H

#include <QStyledItemDelegate>

namespace map
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

} // namespace map

#endif
