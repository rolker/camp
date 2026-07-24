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
  QAction* remove_action = menu->addAction("Remove");
  connect(remove_action, &QAction::triggered, this, [this]()
  {
    removeFromMap();
  });
}

void Layer::removeFromMap()
{
  // [camp#90] Removal (user "Remove" or programmatic): let a persisted layer
  // drop itself from its restore list so it stays removed next session (app
  // shutdown does not call this path, so persisted layers survive a normal quit).
  onRemovedFromMap();
  // [camp#117] Mark removed so a shutdown that races the deleteLater below does not
  // re-persist this layer's settings group (see MapItem::applicationQuitting).
  removed_from_map_ = true;
  // [#59 ADR-0003] Detach through the Map model (fires rowsAboutToBeRemoved so
  // owners can sync their bookkeeping), drop it from the scene so it stops
  // rendering at once, then delete after the current event unwinds.
  if(auto* m = parentMap())
    m->setMapItemParent(this, nullptr);
  if(scene())
    scene()->removeItem(this);
  deleteLater();
}

void Layer::updateFlags(Qt::ItemFlags& flags) const
{
  MapItem::updateFlags(flags);
  flags |= Qt::ItemIsEditable;
  flags |= Qt::ItemIsUserCheckable;
  flags |= Qt::ItemIsDragEnabled;
  // [#84] A leaf layer must NOT be drop-enabled. Reordering happens by dropping
  // a layer onto the gap between rows, which targets the LayerList (the layer's
  // parent) and is handled by Map::dropMimeData. When the drop lands in the
  // middle of a *row*, QAbstractItemView treats it as a drop ONTO that row
  // (OnItem) — but only converts it to an Above/Below (between-row) drop if the
  // target row is not ItemIsDropEnabled (see QAbstractItemViewPrivate::position).
  // A Layer has no working canDropMimeData (MapItem's returns false: layers are
  // not drop containers), so leaving the flag set made every mid-row drop read
  // as a forbidden OnItem drop — reordering only worked in the thin between-row
  // margins, which presented as "drops only land at the first spot". Dropping
  // the flag lets OnItem become a between-row drop everywhere, so the reorder
  // works wherever the row is. Some Layer subclasses (markers, grids) already
  // nest children, but none accept drops, so this is correct for all of them
  // today. A layer that should accept drops *into* it must re-add this flag
  // *and* override canDropMimeData.
}

void Layer::readSettings()
{
  MapItem::readSettings();

  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(settingsKey());

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
  settings.beginGroup(settingsKey());

  settings.setValue("opacity", opacity());
  settings.setValue("visible", isVisible());

  settings.endGroup();
  settings.endGroup();
}


} // namespace map
} // namespace camp
