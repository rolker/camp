#include "layer.h"
#include "layer_list.h"
#include <QGraphicsScene>
#include <QSettings>

namespace map
{

Layer::Layer(MapItem* parent, const QString& object_name):
  MapItem(parent, object_name)
{

}


void Layer::updateFlags(Qt::ItemFlags& flags) const
{
  MapItem::updateFlags(flags);
  flags |= Qt::ItemIsEditable;
  flags |= Qt::ItemIsUserCheckable;
  flags |= Qt::ItemIsDragEnabled;
  flags |= Qt::ItemIsDropEnabled;
}

void Layer::readSettings()
{
  MapItem::readSettings();

  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(objectName());

  setOpacity(settings.value("opacity", 1.0).toReal());
  setVisible(settings.value("visible", true).toBool());

  settings.endGroup();
  settings.endGroup();
}

void Layer::writeSettings()
{
  MapItem::writeSettings();
  
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(objectName());

  settings.setValue("opacity", opacity());
  settings.setValue("visible", isVisible());

  settings.endGroup();
  settings.endGroup();
}


} // namespace map
