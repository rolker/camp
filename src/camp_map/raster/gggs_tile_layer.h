#ifndef RASTER_GGGS_TILE_LAYER_H
#define RASTER_GGGS_TILE_LAYER_H

#include "../map/layer.h"
#include "raster_field_source.h"
#include "raster_gl_renderer.h"

#include <marine_colormap/transfer.hpp>

#include <QFutureWatcher>
#include <QGeoCoordinate>
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
/// tessellation. Multi-band GeoTIFFs expose a per-layer band picker (camp#108).
/// [camp#103 / ADR-0011] paint() renders only the viewport-visible clip of the
/// extent, sized to the on-screen pixels, so zoomed-in renders stay crisp on
/// stores far larger than kMaxImageEdge.
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

  /// [camp#180] Elevation at a geographic point from this store's tiles, or NaN
  /// if no loaded tile covers it (or the covering tiles have no value there).
  /// Among tiles whose extent contains @p location, the finest (highest tile
  /// level, parsed from the `<level>_<row>_<col>` basename) is sampled first, so
  /// inspection prefers the highest-resolution data regardless of the rendered
  /// LOD (camp#103 follow-on). Values are the raw band samples — ellipsoidal
  /// up-positive heights for a bathymetry store, NOT chart-datum depths. A pure
  /// in-memory query (GggsTile::sampleAt over resident pixels); safe to call per
  /// cursor move on the GUI thread.
  float getElevation(const QGeoCoordinate& location) const;

  /// The layer's Web-Mercator extent (union of tile extents). The item is
  /// setPos()'d at its top-left; exposed for tests.
  QRectF sceneBounds() const { return scene_bounds_; }

  /// Warp the tiles into an offscreen image of @p size spanning the layer's
  /// Web-Mercator extent (boundingRect). Returns a null image if there is no
  /// data or GL is unavailable. Exposed so a headless test can render + inspect
  /// without a window.
  QImage renderImage(const QSize& size);

  /// [camp#103 / ADR-0011] Clip-aware overload: warp only the tiles whose scene
  /// extent intersects @p clip_bounds (a sub-rect of sceneBounds(), Web-Mercator
  /// metres) into an image of @p size spanning exactly @p clip_bounds. paint()
  /// uses it with the viewport-derived clip; public (mirroring renderImage(size))
  /// so the headless clip-filter test can call it directly.
  QImage renderImage(const QSize& size, const QRectF& clip_bounds);

  /// [camp#90 / camp#141] Select the colour ramp by marine_colormap palette name
  /// (baked to a GPU LUT). Persists and re-renders. Unknown name -> grayscale.
  void setColormap(const std::string& name);

  /// [camp#108] The 1-indexed band the tiles render (default 1).
  int band() const { return band_; }

  /// [camp#108] Number of bands in the tile-set, taken from the first valid tile
  /// (the tiles of a store are uniform). 0 if there is no valid tile yet.
  int bandCount() const;

  /// [camp#142] Per-layer colormap range override. In Auto mode the resolved
  /// range tracks the data extents (data_min_/data_max_, folded in tilesReady());
  /// in Manual mode it is pinned to an operator-chosen [lo, hi] so an outlier band
  /// (e.g. backscatter max 925, mean 0.16) can't collapse the useful colour range.
  /// The override is applied at render time (fed to the shader's u_min/u_max via
  /// renderToImage) and is transparent to the auto-range accumulation. Each setter
  /// persists and re-renders.
  void setRangeOverride(float lo, float hi);   ///< -> Manual [lo, hi]
  void resetRangeToAuto();                      ///< -> Auto (tracks the data extents)
  marine_colormap::RangeMode rangeMode() const { return range_model_.mode(); }
  float rangeLo() const { return range_model_.lo(); }   ///< current resolved low bound
  float rangeHi() const { return range_model_.hi(); }   ///< current resolved high bound

  /// [camp#108] Select which 1-indexed band the layer renders, then persist it.
  /// Delegates the band switch to applyBand() and round-trips the selection to
  /// QSettings. No-op if @p band is out of range or unchanged.
  void setBand(int band);

  /// [camp#132] Toggle the QPainter blit smoothing for this layer's paint()
  /// (default OFF = Nearest, the faithful-QA baseline). Governs ONLY the blit
  /// hint — the GL scalar data-texture filter stays Nearest always (a Linear
  /// filter would blend the finite NoData sentinel into fabricated values the
  /// shader's exact-equality discard can't catch — the camp#122 halo).
  /// Persists and repaints.
  void setSmoothInterpolation(bool smooth);
  bool smoothInterpolation() const { return smooth_interpolation_; }

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

  /// [camp#103 / ADR-0013] The currently selected LOD level, or -1 when no
  /// selection has been made (headless / never painted): -1 means NO level
  /// filter anywhere — worker, items, range fold — so the pre-LOD behavior
  /// (load and render everything) is preserved bit-for-bit for the existing
  /// headless tests. paint() drives this from the viewport scale.
  int selectedLevel() const { return selected_level_; }

  /// [camp#103] Deduplicated ascending list of GGGS levels present across the
  /// tile-set (fine dir + `overviews/` sidecar). Exposed for tests.
  const std::vector<int>& availableLevels() const { return available_levels_; }

  /// [camp#103] Test-only: force the LOD selection + load viewport that paint()
  /// would normally derive from the view, so a headless test can exercise the
  /// demand-driven filter without a QGraphicsView. A null @p viewport_scene
  /// means no spatial filter.
  void setLodForTest(int level, const QRectF& viewport_scene)
  {
    selected_level_ = level;
    load_viewport_ = viewport_scene;
  }

  /// [camp#103] Test-only: number of tiles at @p level whose pixels are loaded.
  int pixelsLoadedCount(int level) const;

  /// [camp#103] True if any tile at the selected level intersects
  /// @p viewport_scene (Web-Mercator scene rect) with its pixels not yet
  /// loaded — the pan/zoom re-kick condition for the demand-driven loader
  /// (a pure pan must re-kick or panned-in regions stay blank forever). With
  /// no selection (selected_level_ == -1) the level filter is off; a null
  /// viewport means everything is "visible". Public as the paint() helper and
  /// the unit-test seam for the re-kick predicate.
  bool hasUnloadedVisibleTiles(const QRectF& viewport_scene) const;

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
  /// [camp#103/#194] Recompute available_levels_ (dedup ascending) and
  /// scene_bounds_ (union of every NATIVE tile's extent, at any level — a
  /// region-disjoint native ladder needs every level's footprint; overview
  /// sidecar tiles are padded to their coarse GGGS grid cell, so uniting them
  /// would balloon the extent far beyond the data footprint) from tiles_.
  /// Called after any tiles_ mutation (loadDirectory, rescan).
  void rebuildLevelIndex();
  /// [camp#103] items() body with an optional scene-space clip: a non-null
  /// @p clip_scene keeps only tiles whose Web-Mercator extent intersects it,
  /// tested BEFORE the lazy texture upload so offscreen tiles cost nothing.
  /// items() (the RasterFieldSource interface) delegates with a null rect.
  QList<RasterFieldItem> itemsIntersecting(const QRectF& clip_scene);
  /// [camp#102] Worker body (runs off-thread): loadPixels() each not-yet-loaded
  /// tile, honoring abort_flag_ between tiles. GDAL only — never touches GL.
  /// [camp#103] The demand-driven filter travels as VALUE COPIES snapshotted at
  /// kick time (loadTiles()), never as reads of the live members — paint()
  /// reassigns selected_level_/load_viewport_ every frame while a worker may be
  /// running, and QRectF/int member reads from the worker thread would race.
  /// @p level == -1 disables the level filter; a null @p viewport disables the
  /// spatial filter (both together = the pre-LOD "load everything" behavior the
  /// headless tests rely on).
  void loadTilesWorker(int level, QRectF viewport);

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
  bool smooth_interpolation_ = false;   // [camp#132] blit hint only (persisted)
  std::vector<std::unique_ptr<GggsTile>> tiles_;
  QRectF scene_bounds_;        // union of NATIVE tile extents (see rebuildLevelIndex)

  // [camp#103 / ADR-0013] LOD selection state (GUI thread only — the worker gets
  // value copies at kick time, see loadTilesWorker). selected_level_ == -1 =
  // no selection = no filtering anywhere (headless default). last_kick_* record
  // the filter of the most recent kick so an idle re-kick fires only when the
  // selection or viewport actually moved — not every frame while a failed tile
  // sits permanently unloaded (which would repaint-loop forever).
  int selected_level_ = -1;
  std::vector<int> available_levels_;   // dedup ascending, fine + overviews
  QRectF load_viewport_;                // scene-space filter for the next kick
  int last_kick_level_ = -1;
  QRectF last_kick_viewport_;
  double data_min_ = 1.0;      // auto-range over all tiles (crossed => no data)
  double data_max_ = 0.0;

  // [camp#142] Resolved colormap range (Auto tracks data_min_/data_max_ via
  // update_auto() in tilesReady(); Manual pins an operator override). Fed to the
  // renderer's u_min/u_max at render time, replacing the raw data_min_/data_max_.
  marine_colormap::RangeModel range_model_;

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

  // [camp#103] Last render, keyed by FBO size AND viewport clip: zoom changes the
  // size, pan changes the clip, so both re-render. (Pre-#103, pan reused the
  // cached whole-extent image via the world transform; a viewport-sized FBO makes
  // the per-frame re-render cheap and is required for correctness.)
  QImage cached_image_;
  QSize cached_size_;
  QRectF cached_clip_;
};

}  // namespace raster
}  // namespace camp

#endif
