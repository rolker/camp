#ifndef RASTER_GGGS_TILE_LAYER_H
#define RASTER_GGGS_TILE_LAYER_H

#include "../map/layer.h"
#include "raster_field_source.h"
#include "raster_gl_renderer.h"
#include "tile_residency.h"

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

  /// [camp#103 / ADR-0013 / camp#194] The currently selected LOD level, or -1
  /// when no selection has been made (headless / never painted). The
  /// selection is a CEILING, not an equality filter: every available level
  /// <= it loads (viewport-bounded) and stays resident, compositing
  /// coarse→fine so a region-disjoint native ladder renders all its regions
  /// (camp#194); levels above it are excluded from loading and released once
  /// the selection's visible set completes. -1 means NO level filter
  /// anywhere — worker, range fold, release — so the pre-LOD behavior (load
  /// and render everything) is preserved for the existing headless tests.
  /// paint() drives this from the viewport scale.
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

  /// [camp#195] Number of tiles currently holding pixels — the quantity the
  /// residency budget bounds. Cheap (a flag test per tile); exposed for tests
  /// and for diagnostics.
  std::size_t residentTileCount() const;

  /// [camp#195] Test-only: override the residency budget in BYTES (0 disables
  /// eviction entirely — the pre-#195 behaviour). Mirrors
  /// SonarLiveCacheLayer::setResidentBudgetForTest(); the production value comes
  /// from `QSettings GggsTileLayers/max_resident_bytes`.
  void setResidentBudgetBytesForTest(std::size_t bytes)
  {
    resident_budget_bytes_ = bytes;
  }

  /// [camp#195] Test-only: run the residency pass synchronously — the headless
  /// analogue of paint()'s protect-then-schedule plus the queued
  /// evictIfOverBudget() slot. A headless test has no event loop turn between
  /// paints, so it drives the pass directly.
  void refreshResidencyForTest() { evictIfOverBudget(); }

  /// [camp#195] Test-only: run paint()'s HALF of the residency pass — protect
  /// the current frame's set and SCHEDULE the eviction — without a
  /// QGraphicsView. Exercises the debounce and the queued hop, so a test can
  /// assert the deferred path converges after
  /// `QCoreApplication::processEvents()` rather than only the synchronous
  /// refreshResidencyForTest() shortcut.
  void scheduleResidencyForTest() { scheduleEvictionIfNeeded(refreshProtection()); }

  /// [camp#103/#194] True if any tile at a level <= the selected level
  /// intersects @p viewport_scene (Web-Mercator scene rect) with its pixels
  /// not yet loaded — the pan/zoom re-kick condition for the demand-driven
  /// loader (a pure pan must re-kick or panned-in regions stay blank
  /// forever), and tilesReady()'s release gate for the finer-than-selection
  /// zoom-out backdrop (the composited picture is complete only when every
  /// visible tile up to the selection has loaded). With no selection
  /// (selected_level_ == -1) the level filter is off; a null viewport means
  /// everything is "visible". Public as the paint() helper and the unit-test
  /// seam for the re-kick predicate.
  /// [camp#194] Tiles whose read failed terminally (GggsTile::loadFailed())
  /// are EXCLUDED: they will never load, so counting them would pin this
  /// predicate true for the session — wedging both the re-kick guard and the
  /// release gate. tilesReady() surfaces their count through setStatus()
  /// instead, so the layer settles visibly-incomplete rather than silently so.
  /// [camp#194 review] Consequence for the release gate: "no unloaded visible
  /// tiles" means the loader has SETTLED, not that the footprint is covered —
  /// a failed tile leaves a hole. tilesReady() therefore retains finer tiles
  /// intersecting a failed selected-or-coarser tile rather than releasing the
  /// only usable coverage over that hole.
  bool hasUnloadedVisibleTiles(const QRectF& viewport_scene) const;

  /// [camp#102] Re-scan the tile directory for newly-landed `*.tif` files. Adds
  /// extent-only entries for any tile not already held and re-kicks the async
  /// pixel load if the layer is already loaded. A half-written tile that fails to
  /// open degrades to valid()==false and is skipped — never crashes. Returns true
  /// if any tile was added; safe to call repeatedly / when nothing changed.
  /// [camp#194 review] ALSO refreshes tiles whose file was REPLACED at the same
  /// path (GggsTile::fileChangedOnDisk(): size/mtime differ from the last
  /// metadata read) — re-reading their metadata and clearing any latched
  /// GggsTile::loadFailed(), so a tile repaired by a producer (uma
  /// `enc_updater`'s cron chart-layer rewrite, `overview_pyramid`'s
  /// rename-aside swap) recovers without restarting CAMP. Such a refresh also
  /// returns true.
  /// [camp#194 review round 3] EVERY latched GggsTile::loadFailed() tile is a
  /// refresh candidate too, whether or not its file changed: a transient I/O
  /// error (NFS blip) leaves size and mtime identical, and Rescan is an
  /// explicit operator retry, so it must not be gated on a stat that a
  /// transient failure never perturbs. Loaded tiles remain stat-gated (no churn
  /// on a healthy store). A tile that is still unreadable simply re-latches.
  /// Consequently Rescan returns true while any tile stays failed.
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
  /// [camp#195 / uma-ADR-0013 D4] Deferred, debounced residency eviction.
  /// paint() only SCHEDULES this (queued invocation + eviction_pending_):
  /// releasing a tile inside paint() would mutate the set the render pass is
  /// reading. Running from the event loop, it re-derives the protected working
  /// set LIVE (a tile that re-entered the view since scheduling must never be
  /// evicted — the MapTiles/camp#98 rule), computes the cap, and releases
  /// candidates farthest/stalest-first down to the hysteresis target.
  ///
  /// It must not mutate tiles the loader worker is iterating, so it checks
  /// `future_watcher_.isRunning()` — and when the worker IS busy it **re-arms**
  /// on a short timer with the debounce still held, rather than dropping the
  /// request. Dropping it would make the budget almost never fire during a
  /// continuous pan (every pan step re-kicks the loader), which is precisely
  /// the scenario the budget exists for. Abort+join (what loadTiles() does) is
  /// rejected here: it would stall the GUI thread mid-pan for a whole tile read.
  void evictIfOverBudget();

private:
  /// [camp#108] The non-persisting band switch shared by setBand() (persists
  /// after) and readSettings() (applies an already-persisted value, so must not
  /// write it back). Validates against bandCount(), aborts + joins any in-flight
  /// load, releases every tile's GL texture under this layer's context, re-points
  /// each tile at the new band (skipping + WARNing on tiles with too few bands),
  /// resets the layer auto-range, invalidates the cached image, re-kicks the async
  /// load, and repaints. No-op if @p band is out of range or unchanged.
  void applyBand(int band);
  /// [camp#102/#194] Fold the loaded tiles' data ranges into the layer aggregate
  /// data_min_/data_max_ (current band only, levels <= the selection; all-NoData
  /// and failed tiles contribute nothing).
  ///
  /// @p reset selects the two modes. FALSE (tilesReady()'s steady-state fold) is
  /// INCREMENTAL: it starts from the existing aggregate and only ever WIDENS it,
  /// which is deliberate — a finer tile that contributed while it was <= an
  /// earlier selection keeps its contribution across a level switch, and the
  /// MEAN fold guarantees overview values ⊆ the fine range (ADR-0013).
  /// [camp#194 review round 3] TRUE discards the aggregate first and recomputes
  /// it from the CURRENT resident set — required by any caller that made an
  /// already-folded contribution STALE, which widening alone can never undo
  /// (rescan()'s refreshFromFile() replacing a loaded tile's file). Callers that
  /// reset are responsible for pushing the result to range_model_ (a no-op under
  /// a Manual override, which the recompute must never disturb).
  void foldDataRange(bool reset);
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
  /// [camp#194/#195] Scene rects of the terminally-failed tiles at levels <= the
  /// selection — the holes in the composited picture. Empty with no selection.
  std::vector<QRectF> failedFootprints() const;
  /// [camp#194/#195] True if @p tile is finer than the selection and overlaps
  /// one of @p holes, i.e. it is the only usable coverage over a footprint whose
  /// selected-or-coarser tile failed to read. Shared by tilesReady()'s release
  /// gate and the residency budget's eviction pass so the two retention rules
  /// cannot drift apart.
  bool coversHole(const GggsTile& tile, const std::vector<QRectF>& holes) const;
  /// [camp#195] The single tile-release path: `releaseGL()` + `resetPixels()`
  /// under this layer's GL context, with the makeCurrent()-or-skip dance that
  /// keeps the gggs_tile.h pairing invariant intact. GUI thread only, and only
  /// with no loader worker running (it mutates tiles the worker iterates —
  /// both callers gate on that). Returns true if anything was released; false
  /// when there were no victims or a live context refused to become current
  /// (in which case NOTHING is released, deliberately).
  bool releaseTiles(const std::vector<GggsTile*>& victims);
  /// [camp#195] THE status composer. Every condition the layer reports (no
  /// tiles / loading / no data / failed tiles) is assembled here from live
  /// state, so no writer can clobber another's message. Call this instead of
  /// setStatus() — tilesReady() used to rewrite the status unconditionally.
  void updateStatus();
  /// [camp#195 / uma-ADR-0013 D4] Re-derive the current frame's protected
  /// working set into residency_ and return its size. The predicate is the
  /// LOADER's, not the draw list's: tiles intersecting load_viewport_ at a level
  /// <= the selection. It must not be the draw list — `cached_image_`
  /// short-circuits itemsIntersecting() on a static frame, so a
  /// draw-list-sourced protection would protect NOTHING on exactly the frames
  /// eviction runs on. Not-yet-loaded tiles count: they are part of the working
  /// set the cap must accommodate. loadFailed() tiles do not: they never become
  /// resident, so they must not inflate the floor.
  ///
  /// Two classes of in-view tile FINER than the selection are protected on top
  /// of the loader's set, because both are drawn and neither can reload
  /// (loadTilesWorker skips level > selection): an already-RESIDENT finer tile,
  /// which during a zoom-out is the whole visible picture until the coarser
  /// selection loads (the camp#103/#194 no-blank-frame guarantee — and
  /// tilesReady()'s level-switch release still drops it at the right moment, so
  /// this leaks nothing); and a camp#194 hole coverer.
  std::size_t refreshProtection();
  /// [camp#195] The budget expressed as a tile count: the byte budget divided by
  /// the OBSERVED per-tile resident cost. 0 means unbounded (budget disabled, or
  /// no valid tile to measure yet).
  std::size_t budgetTiles() const;
  /// [camp#195] Bytes a loaded tile occupies, observed from the tile-set rather
  /// than assumed: width_/height_ come from GDAL, so 960x960 is a store
  /// convention, not a guarantee. A loaded tile holds its Float32 CPU buffer
  /// (retained past the GPU upload for camp#180's cursor readout) and, once
  /// painted, an R32F texture of the same size — so the displayed cost is
  /// 2 x w x h x 4. The largest tile in the set speaks, so a non-uniform store
  /// cannot undercount. 0 if no valid tile exists yet.
  std::size_t perTileResidentBytes() const;
  /// [camp#195] Update over_budget_ from @p protected_count and queue an
  /// eviction pass when residency exceeds the cap. Called from paint().
  void scheduleEvictionIfNeeded(std::size_t protected_count);

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
  // [camp#195] Load-in-flight flag feeding updateStatus()'s "(loading...)".
  // A flag rather than future_watcher_.isRunning(): tilesReady() runs from the
  // watcher's finished signal, where isRunning() is already false, and
  // waitForLoad() calls tilesReady() directly after a join.
  bool loading_ = false;

  // [camp#103] Last render, keyed by FBO size AND viewport clip: zoom changes the
  // size, pan changes the clip, so both re-render. (Pre-#103, pan reused the
  // cached whole-extent image via the world transform; a viewport-sized FBO makes
  // the per-frame re-render cheap and is required for correctness.)
  QImage cached_image_;
  QSize cached_size_;
  QRectF cached_clip_;

  // ---- [camp#195 / uma-ADR-0013 D4] Viewport-scoped retention -------------
  // Default residency budget in BYTES. A byte target rather than a tile count
  // because width_/height_ come from GDAL (960x960 is a store convention, not a
  // guarantee) and because it is then directly comparable to the co-resident
  // LiveTileCache/max_vram_bytes, which carries the same 512 MiB default.
  // Operator-overridable via QSettings (camp#117: a default, never an
  // un-changeable hardcode); 0 disables eviction (the pre-#195 behaviour).
  static constexpr qulonglong kDefaultResidentBudgetBytes = 512ull * 1024 * 1024;
  // Evict down to this fraction of the cap so the next frame cannot immediately
  // re-trigger (the SonarLiveCacheLayer kReloadHysteresisFactor analogue).
  static constexpr double kEvictHysteresisFactor = 0.75;
  // The zoom-out floor: tiles at the coarsest available level are exempt (the
  // kApexProtectLevel analogue) — but BOUNDED, because that set grows with the
  // area panned. Beyond this many, the farthest coarsest tiles become
  // last-resort candidates rather than exemptions. Only applies to a real
  // ladder: on a single-level store every tile would otherwise be exempt and
  // the budget would be a no-op.
  static constexpr std::size_t kCoarsestExemptCap = 64;
  // A tile seen within this many paints is "recent" and evicts after the stale
  // ones — the staleness term that keeps distance from being the primary key
  // (uma-ADR-0013 D4: distance-only "discards history along a path being
  // traversed"). ~2 s of paints at typical repaint rates.
  static constexpr quint64 kRecentGenerations = 120;
  // Re-arm delay when the eviction pass finds the loader worker busy.
  static constexpr int kEvictionRetryMs = 100;

  TileResidency residency_;             // protected/evictable partition
  std::vector<quint64> tile_last_visible_gen_;   // parallel to tiles_ (LRU key)
  quint64 paint_generation_ = 0;        // bumped by refreshProtection()
  std::size_t resident_budget_bytes_ = 0;   // 0 = eviction disabled
  bool eviction_pending_ = false;       // debounces the queued eviction
  bool eviction_warned_ = false;        // one qWarning per layer, not per pass
  bool over_budget_ = false;            // protected set alone exceeds the budget
  bool hole_coverage_released_ = false; // a camp#194 hole coverer was evicted
  // Set when releaseTiles() refused because a live GL context would not become
  // current. RasterGlRenderer latches that failure without destroying the
  // context, so the condition can be permanent and the budget stops being
  // enforced entirely — which the status must say rather than grow silently.
  bool eviction_blocked_ = false;
  // Memoized perTileResidentBytes(), keyed on tiles_.size() — this is on the
  // paint path. tiles_ only ever grows, so the size covers every ADDED tile.
  // It does NOT cover a refresh: [camp#194] GggsTile::refreshFromFile() re-reads
  // metadata and a replacement file may carry different dimensions while the
  // count stays put, so rescan() invalidates this explicitly after refreshing.
  mutable std::size_t per_tile_bytes_ = 0;
  mutable std::size_t per_tile_bytes_count_ = std::size_t(-1);
};

}  // namespace raster
}  // namespace camp

#endif
