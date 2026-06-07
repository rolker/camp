#include "layer.h"
#include "layer_list.h"
#include "map.h"
#include <QGraphicsScene>
#include <QSettings>
#include <QMenu>

namespace camp
{
namespace map
{

Layer::Layer(MapItem* parent, const QString& object_name):
  MapItem(parent, object_name)
{

}

void Layer::contextMenu(QMenu* menu)
{
  MapItem::contextMenu(menu);
  // [#59 ADR-0003] Detach through the Map model (fires rowsAboutToBeRemoved so
  // owners can sync their bookkeeping), drop it from the scene so it stops
  // rendering at once, then delete after the menu event unwinds.
  QAction* remove_action = menu->addAction("Remove");
  connect(remove_action, &QAction::triggered, this, [this]()
  {
    if(auto* m = parentMap())
      m->setMapItemParent(this, nullptr);
    if(scene())
      scene()->removeItem(this);
    deleteLater();
  });
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
  settings.beginGroup(itemID());

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
  settings.beginGroup(itemID());

  settings.setValue("opacity", opacity());
  settings.setValue("visible", isVisible());

  settings.endGroup();
  settings.endGroup();
}


} // namespace map
} // namespace camp
