#include "map_item_delegate.h"
#include <QDoubleSpinBox>
#include "../map/map_item.h"

namespace map_tree_view
{

MapItemDelegate::MapItemDelegate(QObject *parent):
  QStyledItemDelegate(parent)
{

}

QWidget *MapItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  //return QStyledItemDelegate::createEditor(parent, option, index);

  QDoubleSpinBox *editor = new QDoubleSpinBox(parent);
  editor->setMinimum(0.0);
  editor->setMaximum(1.0);
  editor->setSingleStep(0.05);
  editor->setPrefix("opacity: ");
  return editor;
}

void MapItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
  auto map_item = reinterpret_cast<map::MapItem*>(index.internalPointer());
  auto spin_box = qobject_cast<QDoubleSpinBox*>(editor);
  spin_box->setPrefix(map_item->objectName()+" opacity: ");
  spin_box->setValue(map_item->opacity());
  connect(spin_box, QOverload<double>::of(&QDoubleSpinBox::valueChanged), map_item, &map::MapItem::setOpacity);
  //QStyledItemDelegate::setEditorData(editor, index);
}

void MapItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
  auto map_item = reinterpret_cast<map::MapItem*>(index.internalPointer());
  auto spin_box = qobject_cast<QDoubleSpinBox*>(editor);
  map_item->setOpacity(spin_box->value());
  //QStyledItemDelegate::setModelData(editor, model, index);
}

void MapItemDelegate::updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  QStyledItemDelegate::updateEditorGeometry(editor, option, index);
}

} // namespace map_tree_view
