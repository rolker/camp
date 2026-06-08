#include "layer.h"
#include "layer_list.h"
#include "map.h"
#include <QGraphicsScene>
#include <QSettings>
#include <QMenu>
#include <QWidgetAction>
#include <QSlider>

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

  // [#59] Discoverable per-layer transparency control. The tree delegate already
  // edits opacity, but only via an obscure double-click-to-edit inline spinbox
  // (and it doesn't persist). Expose a labeled 0-100% slider in the right-click
  // menu — consistent with the Colormap submenu on grids/rasters — wired to
  // setOpacity (live) + writeSettings (persists across restarts). On every
  // layer, including the non-removable AIS / Collision / chart layers.
  QMenu* opacity_menu = menu->addMenu("Opacity");
  auto* slider = new QSlider(Qt::Horizontal, opacity_menu);
  slider->setRange(0, 100);
  slider->setValue(static_cast<int>(opacity() * 100.0 + 0.5));
  slider->setMinimumWidth(160);
  connect(slider, &QSlider::valueChanged, this, [this](int value)
  {
    setOpacity(value / 100.0);
    writeSettings();
  });
  auto* slider_action = new QWidgetAction(opacity_menu);
  slider_action->setDefaultWidget(slider);
  opacity_menu->addAction(slider_action);

  if(!removable_)
    return;
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
