#ifndef RASTER_GGGS_TILE_LAYER_H
#define RASTER_GGGS_TILE_LAYER_H

#include "../map/layer.h"
#include "../map/color_map.h"

#include <QFutureWatcher>
#include <QImage>
#include <QMutex>
#include <QSize>
#include <memory>
#include <vector>

class QOpenGLContext;
class QOffscreenSurface;
class QOpenGLFramebufferObject;
class QOpenGLShaderProgram;
class QOpenGLTexture;

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

  /// [camp#90] Select the colour ramp (the shared camp::map::ColorMap, baked to
  /// a GPU LUT). Persists and re-renders.
  void setColormap(map::ColorMap::Type type);

  /// [camp#102] Block until this layer's async pixel load (if any) has completed.
  /// Exposed for headless tests that call renderImage() directly without the
  /// QGraphicsView paint loop that normally kicks + awaits the load via signals.
  void waitForLoad();

  /// [camp#102] Re-scan the tile directory for newly-landed `*.tif` files (for
  /// a caller watching the directory). Adds extent-only entries for
  /// any tile not already held and re-kicks the async pixel load if the layer is
  /// already loaded. A half-written tile that fails to open degrades to
  /// valid()==false and is skipped — never crashes. Returns true if any tile was
  /// added.
  bool rescan();

protected:
  void contextMenu(QMenu* menu) override;
  void readSettings() override;
  void writeSettings() override;
  /// [camp#104] Drop this layer's tile-set directory from the persisted
  /// `GggsTileLayers/dirs` restore list on user removal — the flat-layer
  /// analogue of the retired GggsStoreLayer's root drop. Dedup-on-select
  /// (GggsStoreSource::instantiate) guarantees at most one flat layer per
  /// directory, so this can't orphan a second layer on the same dir.
  void onRemovedFromMap() override;

private slots:
  /// [camp#102] Launch the async pixel read over not-yet-loaded tiles.
  void loadTiles();
  /// [camp#102] Fold completed tiles' ranges into data_min_/data_max_, mark them
  /// pixelsLoaded(), invalidate the cache, and repaint.
  void tilesReady();

private:
  void loadDirectory(const QString& directory);
  /// [camp#102] Worker body (runs off-thread): loadPixels() each not-yet-loaded
  /// tile, honoring abort_flag_ between tiles. GDAL only — never touches GL.
  void loadTilesWorker();
  bool ensureGL();
  bool ensureProgram();
  QOpenGLTexture* ensureLut();
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

  // [camp#102] Async pixel load mirroring RasterLayer (ADR-0003 §3): the cheap
  // extent list is built in loadDirectory(); the band reads are deferred to a
  // single QtConcurrent worker driven by this watcher and kicked lazily from the
  // first paint(). The abort flag + waitForFinished() dtor/re-launch join is
  // ported verbatim from RasterLayer so a layer destroyed mid-load can't outlive
  // `this`. The worker mutates each GggsTile's pixel buffer off-thread and then
  // publishes it by storing the tile's atomic pixelsLoaded() flag with RELEASE
  // ordering (gggs_tile.cpp). The paint path only ever reads a tile's data_/
  // texture() after an ACQUIRE load of that flag returns true (gggs_tile_layer.cpp
  // renderImage). That release/acquire pair establishes happens-before, so the
  // paint thread never observes a half-written buffer — even on the rescan()
  // re-kick, where the layer's crossed-range gate is already open.
  QFutureWatcher<void> future_watcher_;
  bool abort_flag_ = false;
  QMutex abort_flag_mutex_;
  bool load_started_ = false;  // first paint() kicks the load exactly once

  // The layer's own offscreen GL context — created lazily, used only for the
  // FBO render; never touches the GUI's context.
  QOpenGLContext* gl_context_ = nullptr;
  QOffscreenSurface* gl_surface_ = nullptr;
  std::unique_ptr<QOpenGLFramebufferObject> fbo_;
  std::unique_ptr<QOpenGLShaderProgram> program_;
  std::unique_ptr<QOpenGLTexture> lut_texture_;   // colormap LUT (256x1 RGBA)
  bool gl_failed_ = false;

  // Default grayscale preserves the original look; selectable via context menu.
  map::ColorMap colormap_{map::ColorMap::Grayscale};
  bool lut_dirty_ = true;      // re-bake the LUT on next render after a change

  QImage cached_image_;        // last render, reused on pan (re-rendered on zoom)
  QSize cached_size_;
};

}  // namespace raster
}  // namespace camp

#endif
