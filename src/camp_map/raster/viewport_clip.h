#ifndef RASTER_VIEWPORT_CLIP_H
#define RASTER_VIEWPORT_CLIP_H

#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPainter>
#include <QRectF>
#include <QSize>
#include <algorithm>
#include <cmath>

namespace camp
{
namespace raster
{

/// [camp#103 / ADR-0011] The visible sub-region a raster layer should render,
/// derived in paint() from the painter's clip. Shared by the three
/// RasterFieldSource layers (GggsTileLayer, RasterLayer, SonarLiveCacheLayer)
/// so the derivation lives once.
struct ViewportClip
{
  QRectF local;   ///< clip in item-local coords (drawImage target)
  QRectF scene;   ///< clip in Web-Mercator scene coords (renderToImage bounds)
  QSize size;     ///< offscreen FBO size for the clip at the current zoom
};

/// Derive the visible clip of an item whose local space spans
/// (0,0)..(bounding.size()) with the camp_map raster convention: the item is
/// setPos()'d at the NW corner of @p scene_bounds with a fromScale(1,-1)
/// transform, so local y increases southward while scene y increases northward.
///
/// The viewport comes from the attached view — mapToScene(viewport) mapped into
/// the item — because the painter's state is NOT a reliable viewport signal:
/// QStyleOptionGraphicsItem::exposedRect defaults to boundingRect() without
/// ItemUsesExtendedStyleOption (set nowhere in camp), and
/// painter->clipBoundingRect() is only populated on repaint paths where
/// QGraphicsView happens to set a clip (verified empty on live full-viewport
/// repaints — the blur came back in the field). The painter clip is kept as a
/// secondary signal for painter-only render paths; with neither, fall back to
/// the full bounding rect (pre-#103 whole-extent behavior, e.g. headless
/// QA renders). First view only — CAMP has a single MapView.
inline ViewportClip deriveViewportClip(QPainter* painter,
                                       const QGraphicsItem* item,
                                       const QRectF& bounding,
                                       const QRectF& scene_bounds, int max_edge)
{
  ViewportClip clip;
  if(const QGraphicsScene* scene = item ? item->scene() : nullptr)
  {
    const QList<QGraphicsView*> views = scene->views();
    if(!views.isEmpty() && views.first())
    {
      QGraphicsView* view = views.first();
      const QRectF view_scene =
        view->mapToScene(view->viewport()->rect()).boundingRect();
      clip.local = item->mapRectFromScene(view_scene).intersected(bounding);
    }
  }
  if(clip.local.isEmpty())
    clip.local = painter->clipBoundingRect().intersected(bounding);
  if(clip.local.isEmpty())
    clip.local = bounding;

  // Item-local -> scene: x offsets from the west edge; y flips about the north
  // edge (scene_bounds.bottom() is the northern/max-y edge, local y=0).
  const qreal left = scene_bounds.left() + clip.local.left();
  const qreal right = scene_bounds.left() + clip.local.right();
  const qreal top = scene_bounds.bottom() - clip.local.bottom();      // south
  const qreal bottom = scene_bounds.bottom() - clip.local.top();      // north
  clip.scene = QRectF(QPointF(left, top), QPointF(right, bottom));

  // Size the offscreen target to the CLIP's on-screen size (not the whole
  // extent), so zoomed-in renders stay at full pixel density. The max_edge
  // clamp now applies to the viewport, so it is rarely hit.
  const QRectF dev = painter->worldTransform().mapRect(clip.local);
  clip.size = QSize(
    std::min(max_edge, std::max(1, int(std::ceil(std::abs(dev.width()))))),
    std::min(max_edge, std::max(1, int(std::ceil(std::abs(dev.height()))))));
  return clip;
}

}  // namespace raster
}  // namespace camp

#endif
