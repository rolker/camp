#ifndef RASTER_GGGS_TILE_LAYER_H
#define RASTER_GGGS_TILE_LAYER_H

#include "../map/layer.h"

#include <QImage>
#include <QSize>
#include <memory>
#include <vector>

class QOpenGLContext;
class QOffscreenSurface;
class QOpenGLFramebufferObject;
class QOpenGLShaderProgram;

namespace camp
{
namespace raster
{

class GggsTile;

/// [camp#90 / I4] Map layer that renders a directory of GGGS raster tiles
/// (native-geographic WGS84 GeoTIFFs from marine_sidescan_mosaic #173 / the
/// bathymetry store) by warping each tile into the Web-Mercator scene on the
/// GPU — but to an **offscreen framebuffer the layer owns**, then presenting the
/// result with QPainter::drawImage(). This is portable across X11 / Wayland /
/// software GL, independent of whether QGraphicsView paints its viewport through
/// a GL context (it generally does not — native painting in an item gives no
/// current context under a Wayland software backingstore).
///
/// The vertex shader applies web_mercator::geoToMap (the only nonlinearity is
/// the 1-D latitude warp), so the offscreen image is a Web-Mercator raster of
/// the layer's extent; drawing it into boundingRect() (also Web-Mercator) keeps
/// it registered with the vector overlays and CPU raster/tile layers.
///
/// Slice 1: single-band auto-ranged grayscale (sidescan); fixed tessellation;
/// whole-extent render cached by on-screen size. Band-select + colormap are
/// Slice 3 (camp#63 GPU facility); visible-region-only render is Slice 2.
class GggsTileLayer: public map::Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  GggsTileLayer(map::MapItem* parentItem, const QString& directory);
  ~GggsTileLayer();

  enum { Type = map::GggsTileLayerType };
  int type() const override { return Type; }

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

  /// Directory of `<level>_<row>_<col>.tif` tiles this layer renders.
  const QString& directory() const { return directory_; }

  /// True once at least one valid tile loaded.
  bool valid() const { return !tiles_.empty(); }

  /// The layer's Web-Mercator extent (union of tile extents). The item is
  /// setPos()'d at its top-left; exposed for tests.
  QRectF sceneBounds() const { return scene_bounds_; }

  /// Warp the tiles into an offscreen image of @p size spanning the layer's
  /// Web-Mercator extent (boundingRect). Returns a null image if there is no
  /// data or GL is unavailable. Exposed so a headless test can render + inspect
  /// without a window.
  QImage renderImage(const QSize& size);

private:
  void loadDirectory(const QString& directory);
  bool ensureGL();
  bool ensureProgram();
  void releaseGL();

  // Latitude tessellation per tile. The geo->Web-Mercator warp is separable:
  // longitude is linear, latitude is the lone nonlinearity. 16 strips is
  // sub-pixel over a tile at these zooms (Slice 2 tunes this to a < 0.5 px
  // budget and renders only the visible region for large surveys).
  static constexpr int kLatSubdivisions = 16;
  static constexpr int kMaxImageEdge = 4096;   // clamp the offscreen target

  QString directory_;
  std::vector<std::unique_ptr<GggsTile>> tiles_;
  QRectF scene_bounds_;        // union of tile extents in Web-Mercator scene units
  double data_min_ = 1.0;      // auto-range over all tiles (crossed => no data)
  double data_max_ = 0.0;

  // The layer's own offscreen GL context — created lazily, used only for the
  // FBO render; never touches the GUI's context.
  QOpenGLContext* gl_context_ = nullptr;
  QOffscreenSurface* gl_surface_ = nullptr;
  std::unique_ptr<QOpenGLFramebufferObject> fbo_;
  std::unique_ptr<QOpenGLShaderProgram> program_;
  bool gl_failed_ = false;

  QImage cached_image_;        // last render, reused on pan (re-rendered on zoom)
  QSize cached_size_;
};

}  // namespace raster
}  // namespace camp

#endif
