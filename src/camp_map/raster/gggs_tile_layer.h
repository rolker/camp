#ifndef RASTER_GGGS_TILE_LAYER_H
#define RASTER_GGGS_TILE_LAYER_H

#include "../map/layer.h"
#include "raster_field_source.h"
#include "raster_gl_renderer.h"

#include <QFutureWatcher>
#include <QImage>
#include <QMutex>
#include <QSize>
#include <memory>
#include <string>
#include <vector>

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
/// Auto-ranged value mapped through a colormap LUT (camp#90); fixed
/// tessellation; whole-extent render cached by on-screen size. Multi-band
/// GeoTIFFs expose a per-layer band picker (camp#108); visible-region-only
/// render is Slice 2.
class GggsTileLayer: public map::Layer, public RasterFieldSource
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

  /// [camp#126] Per-layer persistence (visible/colormap/band) is keyed on the
  /// tile-set DIRECTORY, not the display name. The display name is parent/leaf
  /// (a label two stores can share, e.g. survey_a/bathymetry/processed and
  /// survey_b/bathymetry/processed both show as "bathymetry/processed"); the
  /// directory is the layer's stable, unique identity. Returns an itemID()-shaped
  /// flat key derived from the absolute directory path so each store gets its own
  /// QSettings group.
  QString settingsKey() const override;

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

  /// [camp#90 / camp#141] Select the colour ramp by marine_colormap palette name
  /// (baked to a GPU LUT). Persists and re-renders. Unknown name -> grayscale.
  void setColormap(const std::string& name);

  /// [camp#108] The 1-indexed band the tiles render (default 1).
  int band() const { return band_; }

  /// [camp#108] Number of bands in the tile-set, taken from the first valid tile
  /// (the tiles of a store are uniform). 0 if there is no valid tile yet.
  int bandCount() const;

  /// [camp#108] Select which 1-indexed band the layer renders, then persist it.
  /// Delegates the band switch to applyBand() and round-trips the selection to
  /// QSettings. No-op if @p band is out of range or unchanged.
  void setBand(int band);

  /// [camp#108] Test-only: the 1-indexed band each held tile is currently set to
  /// read, in tile order. Exposed as the narrow, GL-free seam a headless test uses
  /// to assert applyBand()/rescan() propagated the layer band to EVERY tile (the
  /// per-tile invariant the rendered image can't isolate). Not part of the public
  /// layer surface — for tests only.
  std::vector<int> tileBands() const;

  /// [camp#102] Block until this layer's async pixel load (if any) has completed.
  /// Exposed for headless tests that call renderImage() directly without the
  /// QGraphicsView paint loop that normally kicks + awaits the load via signals.
  void waitForLoad();

  /// [camp#102] Re-scan the tile directory for newly-landed `*.tif` files. Adds
  /// extent-only entries for any tile not already held and re-kicks the async
  /// pixel load if the layer is already loaded. A half-written tile that fails to
  /// open degrades to valid()==false and is skipped — never crashes. Returns true
  /// if any tile was added; safe to call repeatedly / when nothing changed.
  /// [camp#104] Wired to the "Rescan" context-menu action — the manual stopgap
  /// for the live pickup lost with the retired GggsStoreLayer QFileSystemWatcher
  /// (ADR-0005); a per-layer watcher is a follow-up.
  bool rescan();

  // [camp#134] RasterFieldSource: feed the shared RasterGlRenderer. bands() lists
  // the 1-indexed tile bands; items() returns the loaded tiles as Scalar items
  // (textures uploaded lazily under the renderer's current context).
  QStringList bands() const override;
  RasterBandMeta metadata(const QString& band) const override;
  QList<RasterFieldItem> items() override;
  QPair<float, float> dataRange() const override;

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
  /// [camp#108] The non-persisting band switch shared by setBand() (persists
  /// after) and readSettings() (applies an already-persisted value, so must not
  /// write it back). Validates against bandCount(), aborts + joins any in-flight
  /// load, releases every tile's GL texture under this layer's context, re-points
  /// each tile at the new band (skipping + WARNing on tiles with too few bands),
  /// resets the layer auto-range, invalidates the cached image, re-kicks the async
  /// load, and repaints. No-op if @p band is out of range or unchanged.
  void applyBand(int band);
  void loadDirectory(const QString& directory);
  /// [camp#102] Worker body (runs off-thread): loadPixels() each not-yet-loaded
  /// tile, honoring abort_flag_ between tiles. GDAL only — never touches GL.
  void loadTilesWorker();

  // [camp#134] Latitude tessellation moved into RasterGlRenderer (the shared warp).
  static constexpr int kMaxImageEdge = 4096;   // clamp the offscreen target

  // [camp#134] The shared GL raster renderer (its own offscreen context + the
  // unified shader + colormap LUT). Replaces this layer's former duplicated
  // shader/program/LUT/FBO. Default ramp is Grayscale (the renderer's default),
  // preserving the original look; selectable via the context menu. Declared BEFORE
  // the texture-holding tiles_ so reverse-declaration destruction tears the tiles
  // (and their GL textures) down before the renderer's context — the invariant the
  // dtor body already enforces explicitly, now also structural.
  RasterGlRenderer renderer_;

  QString directory_;
  int band_ = 1;               // [camp#108] selected 1-indexed band (persisted)
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

  QImage cached_image_;        // last render, reused on pan (re-rendered on zoom)
  QSize cached_size_;
};

}  // namespace raster
}  // namespace camp

#endif
