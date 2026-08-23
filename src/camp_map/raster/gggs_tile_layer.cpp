#include "gggs_tile_layer.h"

#include "gggs_tile.h"
#include "gggs_tile_util.h"
#include "lod_level_selector.h"
#include "colormap_range_dialog.h"
#include "viewport_clip.h"
#include "../map_view/web_mercator.h"

#include <marine_colormap/colormap.hpp>
#include <marine_colormap/palette.hpp>

#include <QAction>
#include <QDir>
#include <QFileInfo>
#include <QGeoCoordinate>
#include <QMenu>
#include <QSet>
#include <QThread>
#include <QSettings>
#include <QTimer>
#include <QOpenGLTexture>
#include <QPainter>
#include <QTransform>
#include <QUrl>
#include <QtConcurrent>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace camp
{
namespace raster
{

namespace
{

// [camp#134] The kVertex/kFragment shaders, ensureProgram(), ensureLut() and the
// per-vertex geo->Web-Mercator tessellation that used to live here moved into the
// shared raster::RasterGlRenderer (see ADR-0007). The unified shader there also
// fixes the NaN NoData discard (`v != v`). This layer now only collects loaded
// tiles into RasterFieldItems and delegates the draw.

// [camp#126] Tree-view display name for a flat store layer: the last two path
// components ("parent/leaf", e.g. "sidescan/processed"), falling back to just the
// leaf when the directory has no parent component (a root-level dir like
// "/processed"). Two stores that share a leaf (.../sidescan/processed and
// .../bathymetry/processed) stay distinguishable in the tree. This is a DISPLAY
// name only — directory_, persistence, and dedup all stay keyed by the full
// directory (see loadDirectory()/writeSettings()).
QString displayName(const QString& directory)
{
  const QDir dir(directory);                                // QDir trims a trailing slash
  const QString leaf = dir.dirName();
  const QString parent = QFileInfo(dir.path()).dir().dirName();
  return (parent.isEmpty() || parent == ".") ? leaf : parent + '/' + leaf;
}

// [camp#103] A tile's extent in Web-Mercator scene units. geoToMap is monotonic
// in both lon and lat, so the two opposite geographic corners bound the rect.
// Pure math on immutable tile extents — safe from any thread (the worker uses
// it for the demand-driven spatial filter).
QRectF tileSceneRect(const camp::raster::GggsTile& tile)
{
  const QPointF lo = web_mercator::geoToMap(
    QGeoCoordinate(tile.minLat(), tile.minLon()));
  const QPointF hi = web_mercator::geoToMap(
    QGeoCoordinate(tile.maxLat(), tile.maxLon()));
  return QRectF(lo, hi).normalized();
}

// [camp#195] Squared distance from @p point to the nearest point of @p rect —
// 0 while the point is inside. The eviction metric (farthest-from-the-viewport-
// centre first), squared to avoid a sqrt in the sort comparator.
double rectDistanceSquared(const QRectF& rect, const QPointF& point)
{
  const double dx = std::max({rect.left() - point.x(), 0.0,
                              point.x() - rect.right()});
  const double dy = std::max({rect.top() - point.y(), 0.0,
                              point.y() - rect.bottom()});
  return dx * dx + dy * dy;
}

}  // namespace

GggsTileLayer::GggsTileLayer(map::MapItem* parentItem, const QString& directory):
  map::Layer(parentItem, displayName(directory)),
  // [camp#126] Canonicalize to a single absolute form so directory_ — the basis
  // for dedup (directory()), persistence (the GggsTileLayers/dirs list and
  // settingsKey()), and de-persist (onRemovedFromMap()) — is identical for any
  // string variant of the same path (trailing slash / relative). The store
  // source canonicalizes too; this defends the other caller (createDefaultLayers
  // restoring from the dirs list). loadDirectory()/rescan() use QDir(directory_),
  // which is path-agnostic, so the absolute path works unchanged.
  directory_(QDir(directory).absolutePath())
{
  // [camp#195 / uma-ADR-0013 D4] Resident tile-footprint budget for eviction.
  // Default 512 MiB, operator-overridable (camp#117: a default, never an
  // un-changeable hardcode); 0 disables eviction (the pre-#195 unbounded
  // behaviour). Mirrors LiveTileCache/max_vram_bytes, whose default is the same
  // 512 MiB — the two working sets are co-resident, and the key exists so the
  // pair can be tuned on the operator station rather than in a rebuild.
  QSettings settings;
  bool budget_ok = false;
  const qulonglong budget_value =
    settings.value("GggsTileLayers/max_resident_bytes",
                   kDefaultResidentBudgetBytes).toULongLong(&budget_ok);
  if(budget_ok)
    resident_budget_bytes_ = static_cast<std::size_t>(budget_value);
  else
  {
    // A malformed value ("512M", "512 MiB", a stray quote) converts to 0, which
    // is the DISABLE sentinel — a typo in the hand-edited operator-station
    // settings would otherwise silently restore the pre-#195 unbounded
    // behaviour. Fall back to the default and say so.
    qWarning().noquote()
      << "[gggs" << directory_
      << "] GggsTileLayers/max_resident_bytes is not a number ("
      << settings.value("GggsTileLayers/max_resident_bytes").toString()
      << ") - using the" << qulonglong(kDefaultResidentBudgetBytes)
      << "byte default. Set it to 0 to disable eviction deliberately.";
    resident_budget_bytes_ = static_cast<std::size_t>(kDefaultResidentBudgetBytes);
  }

  // [camp#102] tilesReady() folds completed tiles' ranges + repaints on the GUI
  // thread when the async pixel load finishes.
  connect(&future_watcher_, &QFutureWatcher<void>::finished, this,
          &GggsTileLayer::tilesReady);
  // [camp#181 / ADR-0015] The anchor holder is the source-agnostic seam (manual
  // today; chart datum / platform tide later). When the resolved anchor moves,
  // drop the cached image, recompose the status, and repaint. The renderer is fed
  // the resolved value at renderImage() time, not here.
  //
  // This slot is not itself a paint-time path, but updateStatus() is NOT reachable
  // only from outside paint(): paint() -> scheduleEvictionIfNeeded() -> updateStatus()
  // fires whenever the over-budget flag flips. That path is pre-existing (camp#195)
  // and untouched here; the anchor part is composed from live holder state, so it is
  // correct on that path too.
  connect(&shoreline_anchor_, &ShorelineAnchor::changed, this, [this]()
  {
    cached_image_ = QImage();
    updateStatus();
    update(boundingRect());
  });
  // [camp#103] Scan via the CANONICALIZED directory_ (not the raw parameter):
  // rescan()'s known-path dedup compares against these initial tile paths, so
  // both scans must build paths from the same directory string — a raw
  // relative/trailing-slash form here would make every rescan re-add every
  // tile as a duplicate.
  loadDirectory(directory_);
  if(!tiles_.empty())
  {
    // Match the camp_map raster convention (RasterLayer / MapTiles / grids): a
    // NORTH-UP image anchored at the NW corner with a negative-Y item transform.
    // The MapView applies its own scale(s, -s); composed with this fromScale(1,
    // -1) the net Y is positive, so the image draws unmirrored and registers.
    // (An identity transform + a south-up image relies on the view to mirror the
    // image via drawImage, which misregisters it — the ~49 m latitude shift.)
    // Local coordinates stay small (the extent, ~hundreds of m), so QPainter
    // keeps precision at these large Web-Mercator positions.
    setTransform(QTransform::fromScale(1.0, -1.0));
    setPos(QPointF(scene_bounds_.left(), scene_bounds_.bottom()));   // NW corner
  }
  updateStatus();
}

GggsTileLayer::~GggsTileLayer()
{
  // [camp#102] Abort + join any in-flight pixel load BEFORE tearing down GL or
  // dropping the tiles the worker is reading — verbatim from RasterLayer's dtor
  // contract (raster_layer.cpp:38-44), so a layer destroyed mid-load can't
  // outlive `this` (the worker captures `this` and mutates tiles_).
  abort_flag_mutex_.lock();
  abort_flag_ = true;
  abort_flag_mutex_.unlock();
  future_watcher_.waitForFinished();
  // [camp#134] Release each tile's GL texture under the renderer's context (the
  // textures are owned by the GggsTiles, not the renderer). The renderer's own
  // program/LUT/FBO/context are freed by its destructor right after this.
  if(renderer_.makeCurrent())
  {
    for(auto& tile : tiles_)
      tile->releaseGL();
    renderer_.doneCurrent();
  }
}

void GggsTileLayer::loadDirectory(const QString& directory)
{
  // [camp#103 / ADR-0013] Scan the fine tiles AND the derived `overviews/`
  // sidecar (uma ADR-0011: flat dir, same `<level>_<row>_<col>.tif` grammar).
  // All tiles share tiles_; their filename-parsed level() distinguishes them.
  const QDir fine_dir(directory);
  const QDir overview_dir(directory + "/overviews");
  for(const bool overview : {false, true})
  {
    const QDir& dir = overview ? overview_dir : fine_dir;
    if(!dir.exists())
      continue;
    const QStringList files = dir.entryList(QStringList() << "*.tif" << "*.tiff",
                                            QDir::Files, QDir::Name);
    for(const QString& name : files)
    {
      // [camp#112] Skip companion tiles (`_time`/`_source`): the `*.tif` glob
      // also matches them, but only the base value tile is renderable.
      if(!isValueTile(name))
        continue;
      // [camp#102] Extent/metadata only — the GggsTile ctor no longer reads
      // pixels. boundingRect()/sceneBounds() are valid immediately
      // (fit-to-extent works at load time); the band reads (and therefore the
      // data range) are deferred to the async loadTiles() worker, so
      // data_min_/data_max_ accumulate incrementally in tilesReady().
      auto tile = std::make_unique<GggsTile>(dir.filePath(name));
      if(!tile->valid())
        continue;
      // [camp#194] Drop a tile whose level failed to parse. -1 is the layer's
      // NO-SELECTION sentinel (selected_level_ == -1 disables the ceiling
      // everywhere), so a tile carrying it as a real level is indistinguishable
      // from "no filter": it would enter available_levels_, and the first
      // viewport whose ideal level is coarser than every other level would make
      // selectLodLevel() return -1 — silently reverting to the eager
      // whole-store load ADR-0013 exists to prevent. isValueTile() already
      // requires three digit groups, so the only way to get here is a digit
      // string too long for int (tileLevel()'s overflow -> -1); no GGGS
      // producer emits one (levels are 0-20), which is exactly why such a name
      // must be rejected rather than folded into the ladder.
      if(tile->level() < 0)
      {
        qWarning("GggsTileLayer: skipping '%s' — unparsable tile level",
                 qUtf8Printable(name));
        continue;
      }
      // [camp#194] Tag sidecar provenance: overview tiles are padded to their
      // coarse GGGS grid cell, so rebuildLevelIndex() excludes them from the
      // scene-bounds union; native tiles at ANY level are the true footprint.
      tile->setOverview(overview);
      tiles_.push_back(std::move(tile));
    }
  }
  rebuildLevelIndex();
}

void GggsTileLayer::rebuildLevelIndex()
{
  // [camp#103] Deduplicated ascending level list + the layer extent.
  //
  // [camp#194] scene_bounds_ unions every NATIVE (non-overview) tile's extent,
  // at ANY level — not the finest level only. A region-disjoint native ladder
  // (ENC chart store, uma ADR-0010 D7: one native level per compilation
  // scale, each covering only its own sub-region) needs every level's
  // footprint in the union, or the regions outside the finest level's
  // coverage sit outside boundingRect() and can never paint (QGraphicsView
  // culls there regardless of any render-side fix). Overview-sidecar tiles
  // stay excluded: they are padded to their (coarse) GGGS grid cell — the L0
  // apex spans a whole 8-degree grid — so uniting them would balloon
  // boundingRect/fit-to-extent far beyond the data footprint (the hazard the
  // previous finest-level-only union guarded against). For the legacy
  // single-native-level store + overviews/ sidecar the two unions are
  // identical.
  available_levels_.clear();
  for(const auto& tile : tiles_)
  {
    const int level = tile->level();
    auto it = std::lower_bound(available_levels_.begin(), available_levels_.end(), level);
    if(it == available_levels_.end() || *it != level)
      available_levels_.insert(it, level);
  }
  scene_bounds_ = QRectF();
  if(available_levels_.empty())
    return;
  bool first_extent = true;
  for(const auto& tile : tiles_)
  {
    if(tile->isOverview())
      continue;
    const QRectF tile_rect = tileSceneRect(*tile);
    scene_bounds_ = first_extent ? tile_rect : scene_bounds_.united(tile_rect);
    first_extent = false;
  }
  if(!first_extent)
    return;
  // Degenerate store: overview tiles only, no native tile at all. Fall back
  // to the finest level present so the layer keeps an extent (matching the
  // old finest-level-only behavior for this case) instead of a null
  // boundingRect that would silently blank the layer.
  const int finest = available_levels_.back();
  for(const auto& tile : tiles_)
  {
    if(tile->level() != finest)
      continue;
    const QRectF tile_rect = tileSceneRect(*tile);
    scene_bounds_ = first_extent ? tile_rect : scene_bounds_.united(tile_rect);
    first_extent = false;
  }
}

bool GggsTileLayer::rescan()
{
  // [camp#102] Incremental add of newly-landed tiles. [camp#104] Invoked by the
  // "Rescan" context-menu action — the manual stopgap for the live pickup lost
  // with the retired GggsStoreLayer's QFileSystemWatcher (ADR-0005).
  //
  // [camp#104] Compute the new-tile set FIRST, before disturbing any in-flight
  // load. Building `known` and constructing the candidate GggsTiles only READS
  // tiles_ (paths are immutable post-construction) and reads tile metadata off
  // disk — neither mutates tiles_, so it races nothing the worker does. We abort +
  // join the worker ONLY when there is at least one tile to add. A Rescan that
  // finds nothing new must leave the in-flight initial load running untouched:
  // aborting it here (whole-tile granularity, never re-kicked because there is
  // nothing to add) would strand those tiles at pixelsLoaded()==false forever —
  // a silently half-blank layer with no recovery, even though status reads loaded.
  // [camp#194 review] Same read-only first pass collects the tiles whose FILE
  // CHANGED under us (size/mtime differ from the stat taken at their last
  // metadata read). A tile whose loadPixels() failed is otherwise latched for
  // the session — rescan()'s known-path dedup skips its path, the worker skips
  // loadFailed() tiles, and a single-band store never calls setBand() — so a
  // producer that REPAIRS the tile (uma `enc_updater`'s cron rewrite of the
  // chart layer, `overview_pyramid`'s rename-aside swap, or an NFS blip that
  // simply passes) would leave the region blank until CAMP restarts. Detecting
  // the swap here gives the operator a same-band retry through the existing
  // Rescan affordance. Not limited to failed tiles: a rewritten tile that DID
  // load is also stale (it is still serving the old file's pixels), and the
  // refresh re-reads it.
  //
  // [camp#194 review round 3] The file-stat test alone does NOT cover the case
  // the sticky-latch finding was raised against in the first place: a TRANSIENT
  // I/O error (an NFS blip failing GDALOpen()/RasterIO() on a file nobody
  // touched) leaves size and mtime unchanged, so a stat-gated refresh never
  // fires, the worker keeps skipping the latched tile, and every Rescan returns
  // false — the failure is still permanent for the session. Rescan is an
  // EXPLICIT operator action ("I fixed it, try again"), not a background poll,
  // so a latched failure is a retry candidate INDEPENDENTLY of the stat: retry
  // cost is one operator-requested re-read, the cost of not retrying is a
  // permanently blank region. The stat check is kept for the other half of the
  // contract — an already-LOADED tile is only re-read when its file actually
  // changed, so Rescan never churns a healthy resident store.
  QSet<QString> known;
  std::vector<GggsTile*> changed;
  for(const auto& tile : tiles_)
  {
    known.insert(tile->path());
    if(tile->loadFailed() || tile->fileChangedOnDisk())
      changed.push_back(tile.get());
  }

  QDir dir(directory_);
  const QStringList files = dir.entryList(QStringList() << "*.tif" << "*.tiff",
                                          QDir::Files, QDir::Name);
  std::vector<std::unique_ptr<GggsTile>> new_tiles;
  for(const QString& name : files)
  {
    // [camp#112] Skip companion tiles (`_time`/`_source`) before the known check
    // so a rescan never adds them as renderable tiles.
    if(!isValueTile(name))
      continue;
    const QString path = dir.filePath(name);
    if(known.contains(path))
      continue;
    // A half-written tile degrades to valid()==false here and is skipped — the
    // next Rescan re-tries it once the producer's write completes.
    // Producer-side atomic-write safety is uma#189.
    auto tile = std::make_unique<GggsTile>(path);
    if(!tile->valid())
      continue;
    // [camp#194] Same -1-sentinel guard as loadDirectory() (see the rationale
    // there): a tile whose level parses to the no-selection sentinel must never
    // enter available_levels_.
    if(tile->level() < 0)
    {
      qWarning("GggsTileLayer: skipping '%s' — unparsable tile level",
               qUtf8Printable(name));
      continue;
    }
    new_tiles.push_back(std::move(tile));
  }

  if(new_tiles.empty() && changed.empty())
    return false;   // nothing new/changed — leave any in-flight load untouched

  // [camp#102] Now that there IS something to add, abort + join any in-flight load
  // before mutating tiles_ (the worker captures `this` and iterates tiles_, so a
  // push_back reallocation under it would be a use-after-free). Abort granularity
  // is WHOLE-TILE (the worker checks abort_flag_ only between tiles, not mid-
  // RasterIO like RasterLayer's per-scanline check), so a large in-flight tile's
  // RasterIO blocks this GUI-thread join until that one tile finishes. Acceptable
  // at the Massabesic store scale this lands against; finer (sub-tile) abort is a
  // follow-up if tiles grow large. The new tiles' pixels are re-kicked below.
  if(future_watcher_.isRunning())
  {
    abort_flag_mutex_.lock();
    abort_flag_ = true;
    abort_flag_mutex_.unlock();
    future_watcher_.waitForFinished();
  }

  // [camp#194 review] Refresh the swapped-under-us tiles: release each one's GL
  // texture under this layer's context FIRST (the resetPixels() pairing
  // INVARIANT — a CPU-only clear would leave the old texture shadowing the
  // re-read pixels), then re-read the metadata + clear any latched failure so
  // the kick below re-reads the current file. Context handling mirrors
  // applyBand()/tilesReady(): no context yet (hasContext() == false) means no
  // tile can hold a texture, so the CPU half alone is the complete release; a
  // context that EXISTS but fails makeCurrent() means textures may exist and
  // cannot be freed, so skip the refresh entirely rather than break the pairing
  // (the renderer has latched its GL-failed flag and is drawing nothing anyway;
  // the next Rescan retries, and fileChangedOnDisk() still reports true because
  // no re-stat happened).
  const bool refresh_ok = !changed.empty() &&
    (!renderer_.hasContext() || renderer_.makeCurrent());
  if(refresh_ok)
  {
    for(auto* tile : changed)
    {
      if(renderer_.hasContext())
        tile->releaseGL();
      if(!tile->refreshFromFile())
        qWarning("GggsTileLayer: '%s' cannot be re-read",
                 qUtf8Printable(tile->path()));
    }
    if(renderer_.hasContext())
      renderer_.doneCurrent();

    // [camp#194 review round 3] refreshFromFile() DROPPED each refreshed tile's
    // pixels and range, and the replacement file may carry an entirely different
    // one. tilesReady()'s fold only ever WIDENS the aggregate, so without an
    // explicit invalidation the departed file's extremes would stay in
    // data_min_/data_max_ for the rest of the session: replace the sole [1, 10]
    // tile with a [100, 110] one and Auto renders [1, 110]; replace it with an
    // all-NoData tile and the layer draws blank against a stale, non-crossed
    // range with a clear status. On a bathymetry display that is a wrong
    // colormap range the operator reads as real depth. Recompute the aggregate
    // from the CURRENT resident set instead (the refreshed tiles contribute
    // nothing until their pixels re-load, then re-widen it through tilesReady()).
    // update_auto() is a no-op while the operator holds a Manual override, so a
    // pinned range survives the recompute untouched (camp#142).
    // If the recompute leaves the aggregate CROSSED (every resident tile was
    // refreshed, or the replacements are all-NoData) update_auto() is skipped —
    // as in tilesReady() and applyBand(), the resolved Auto bounds simply hold
    // their last values. Nothing is drawn against them: renderImage() bails on a
    // crossed aggregate and tilesReady() sets the "(no data)" status, so the
    // operator sees an explicitly empty layer rather than a plausible-looking
    // wrong one.
    // NOTE camp#138 tracks the same only-widens class of staleness for
    // SonarLiveCacheLayer; that layer is deliberately NOT touched here.
    foldDataRange(true);
    if(data_min_ <= data_max_)
      range_model_.update_auto(float(data_min_), float(data_max_));
    // [camp#195 review] refreshFromFile() re-reads metadata, so a replacement
    // file may carry different dimensions. perTileResidentBytes() is memoized
    // on tiles_.size(), which a refresh does not change — invalidate it here or
    // the byte budget keeps deriving its tile cap from the departed file's
    // extent.
    per_tile_bytes_count_ = std::size_t(-1);
    cached_image_ = QImage();
  }
  else if(!changed.empty())
    qWarning("GggsTileLayer: %d changed tile(s) not refreshed — no current GL "
             "context to release their textures under", int(changed.size()));

  for(auto& tile : new_tiles)
  {
    // [camp#108] A freshly-constructed GggsTile defaults to band 1; inherit the
    // layer's current band so a tile discovered by a rescan AFTER a band switch
    // (band_ > 1) reads the selected band — not the wrong band scaled through the
    // selected band's range — and folds into the auto-range (tilesReady() folds
    // only tiles whose band() == band_, so a default-band-1 tile would be
    // permanently excluded with no recovery). setBand(1) on the band-1 default is
    // a no-op (GggsTile::setBand returns early on band == band_), so the common
    // band-1 case is unaffected.
    tile->setBand(band_);
    tiles_.push_back(std::move(tile));
  }
  // [camp#103] Wholesale re-index: a rescan can add tiles at a new (finer)
  // level, which both extends available_levels_ and re-bases scene_bounds_
  // (native-tile union — see rebuildLevelIndex). The item pos is DERIVED
  // state of scene_bounds_, so re-anchor unconditionally: a west/north
  // extension (or a finest-level re-base) moves the NW corner, and keeping
  // the old pos would leave the added footprint outside boundingRect() —
  // clipped and unpaintable (Copilot review, PR #183).
  prepareGeometryChange();
  rebuildLevelIndex();
  if(!scene_bounds_.isNull())
  {
    setTransform(QTransform::fromScale(1.0, -1.0));
    setPos(QPointF(scene_bounds_.left(), scene_bounds_.bottom()));
  }

  // [camp#195] Rescan is the operator's explicit retry, and it is the action the
  // released-hole-coverage status names. Clear that report here so it describes
  // the CURRENT state rather than latching for the session: the load kicked
  // below re-reads whatever the refreshed tile-set now admits, and the eviction
  // pass will re-raise the flag if it has to release coverage again.
  hole_coverage_released_ = false;

  if(load_started_)
  {
    cached_image_ = QImage();   // force a re-render once the new pixels arrive
    loadTiles();                // stream the new tiles' pixels in
  }
  return true;
}

void GggsTileLayer::loadTiles()
{
  // [camp#102] Abort + join any in-flight load before launching a new one, then
  // re-arm — verbatim from RasterLayer::loadFile's re-launch guard
  // (raster_layer.cpp:103-123). setFuture() only tracks the latest future, so a
  // replaced job would otherwise keep running untracked, race the new job on
  // tiles_, and — if the layer is destroyed first — outlive `this`. A paint()
  // lazy-kick + a tilesReady() fold therefore cannot launch a second concurrent
  // load.
  //
  // [camp#102] Abort granularity is WHOLE-TILE: the worker honors abort_flag_ only
  // between tiles, so a large in-flight tile's RasterIO blocks this GUI-thread join
  // until that tile finishes (vs RasterLayer's per-scanline abort). Acceptable at
  // current store scale; finer sub-tile abort is a follow-up if tiles grow large.
  if(future_watcher_.isRunning())
  {
    abort_flag_mutex_.lock();
    abort_flag_ = true;
    abort_flag_mutex_.unlock();
    future_watcher_.waitForFinished();
  }
  abort_flag_mutex_.lock();
  abort_flag_ = false;   // re-arm for the new job
  abort_flag_mutex_.unlock();

  loading_ = true;
  updateStatus();
  // [camp#103] Snapshot the demand-driven filter into value copies the worker
  // owns — paint() reassigns the live members every frame while the worker runs,
  // so member reads from the worker thread would race. Record the kick's filter
  // so paint()'s idle re-kick fires only when selection/viewport actually moved
  // (a tile that permanently fails to load must not re-kick every frame).
  last_kick_level_ = selected_level_;
  last_kick_viewport_ = load_viewport_;
  future_watcher_.setFuture(QtConcurrent::run(
    this, &GggsTileLayer::loadTilesWorker, selected_level_, load_viewport_));
}

void GggsTileLayer::loadTilesWorker(int level, QRectF viewport)
{
  // [camp#102] Off-thread: GDAL RasterIO only — NEVER touch GL here (texture()/
  // allocateStorage stay on the paint path). The abort check is between tiles
  // (whole-tile granularity, vs RasterLayer's scanline granularity). loadPixels()
  // writes each tile's data_/range and then publishes it with a RELEASE store to
  // the tile's atomic pixelsLoaded() flag. The paint path reads that flag with an
  // ACQUIRE load before touching data_/texture(), so the release/acquire pair —
  // not the tilesReady() join — is what guarantees the paint thread never reads a
  // tile mid-write. (tilesReady() still runs post-join to fold the range +
  // repaint, but a paint() that races an in-flight worker is already safe.)
  //
  // [camp#103 / ADR-0013] Demand-driven: only tiles at levels UP TO the
  // selected level that intersect the load viewport are read — this is what
  // turns the 3.6 GB eager whole-store open into a viewport-bounded load.
  // [camp#194] The level gate is a CEILING, not an equality: a region-disjoint
  // native ladder (ENC chart store) needs every level <= the selection loaded
  // so the coarser levels' regions composite under the selected level
  // (itemsIntersecting). Levels finer than the selection stay excluded — the
  // demand-driven bound. @p level == -1 (no selection: headless tests,
  // pre-first-paint) disables the level filter and a null @p viewport disables
  // the spatial filter, preserving the pre-LOD load-everything behavior
  // exactly.
  //
  // [camp#194 review] Read coarse levels FIRST. tiles_ is in directory scan
  // order (alphabetical: fine native files sort before coarse ones, and
  // overviews/ is scanned last), which would queue the coarse fill behind
  // large fine reads on a freshly exposed region — on a slow/NFS store the
  // region trickles in at fine resolution with no coarse backdrop. A stable
  // ascending-by-level pass restores coarse-first progressive refinement.
  // Sorting here is off the GUI hot path (worker thread), and tiles_ cannot
  // be mutated while this worker runs (every mutator aborts + joins first),
  // so the raw pointers are safe.
  std::vector<GggsTile*> order;
  order.reserve(tiles_.size());
  for(auto& tile : tiles_)
    order.push_back(tile.get());
  std::stable_sort(order.begin(), order.end(),
                   [](const GggsTile* a, const GggsTile* b)
                   { return a->level() < b->level(); });
  for(auto* tile : order)
  {
    {
      QMutexLocker lock(&abort_flag_mutex_);
      if(abort_flag_)
        return;
    }
    if(tile->pixelsLoaded())
      continue;
    // [camp#194] Skip a tile whose read already failed terminally — otherwise
    // the worker re-opens + re-RasterIOs the dead tile on EVERY kick, i.e. once
    // per pan step (see GggsTile::loadFailed()).
    if(tile->loadFailed())
      continue;
    if(level != -1 && tile->level() > level)
      continue;
    if(!viewport.isNull() && !tileSceneRect(*tile).intersects(viewport))
      continue;
    tile->loadPixels();
  }
}

bool GggsTileLayer::hasUnloadedVisibleTiles(const QRectF& viewport_scene) const
{
  // [camp#103] The pan/zoom re-kick predicate (see header). GUI thread.
  // [camp#194] Ceiling semantics matching the worker: any level <= the
  // selection counts — the composited picture is complete only when every
  // visible tile at every level up to the selection has loaded (this is also
  // tilesReady()'s release gate for the finer-than-selection backdrop).
  for(const auto& tile : tiles_)
  {
    if(tile->pixelsLoaded())
      continue;
    // [camp#194] A tile whose read failed terminally is NOT "still loading".
    // Counting it would leave this predicate true forever: the pan/zoom re-kick
    // guard would keep firing and — the sharper failure — tilesReady()'s
    // load-before-release gate would never open, so the finer-than-selection
    // zoom-out backdrop would stay resident (and excluded from the auto-range
    // fold, rendering clipped) for the rest of the session. The ceiling
    // semantics widen the exposure from "a failing tile AT the selection" to
    // "any failing tile at any level <= the selection", at every zoom.
    if(tile->loadFailed())
      continue;
    if(selected_level_ != -1 && tile->level() > selected_level_)
      continue;
    if(!viewport_scene.isNull() && !tileSceneRect(*tile).intersects(viewport_scene))
      continue;
    return true;
  }
  return false;
}

int GggsTileLayer::pixelsLoadedCount(int level) const
{
  // [camp#103] Test-only seam (see header).
  int count = 0;
  for(const auto& tile : tiles_)
    if(tile->level() == level && tile->pixelsLoaded())
      ++count;
  return count;
}

void GggsTileLayer::foldDataRange(bool reset)
{
  // [camp#102] Fold each loaded tile's range into the layer auto-range (the
  // range is unknown until a tile's pixels load — an all-NoData tile reports a
  // crossed range and is skipped).
  // [camp#194 review round 3] `reset` picks the two modes (see the header):
  // false = the incremental widening fold tilesReady() has always done;
  // true = discard the accumulated aggregate first and recompute it from the
  // CURRENT resident set, for callers that made a tile's contribution stale
  // (rescan()'s refreshFromFile()).
  if(reset)
  {
    data_min_ = 1.0;
    data_max_ = 0.0;
  }
  bool first_range = (data_min_ > data_max_);
  for(auto& tile : tiles_)
  {
    // [camp#108] Only fold tiles that actually carry the layer's current band.
    // A non-uniform tile-set leaves a tile lacking that band on its prior band
    // (applyBand() keeps it loaded rather than blanking it); its stale prior-band
    // min/max must not pollute the current band's auto-range.
    if(tile->band() != band_)
      continue;
    // [camp#103/#194] Tiles finer than the selection must not pollute the
    // auto-range (-1 = no selection = fold everything, the headless default).
    // The fold covers exactly the composited steady-state render set (every
    // level <= the selection); a transiently-resident finer backdrop tile
    // already contributed while it was <= an earlier selection, and the fold
    // only ever WIDENS the range across level switches — acceptable because
    // the MEAN fold guarantees overview values ⊆ the fine range (ADR-0013).
    if(selected_level_ != -1 && tile->level() > selected_level_)
      continue;
    if(!tile->pixelsLoaded() || tile->dataMin() > tile->dataMax())
      continue;
    if(first_range || tile->dataMin() < data_min_) data_min_ = tile->dataMin();
    if(first_range || tile->dataMax() > data_max_) data_max_ = tile->dataMax();
    first_range = false;
  }
}

std::vector<QRectF> GggsTileLayer::failedFootprints() const
{
  // [camp#194/#195] Scene rects of the tiles at levels <= the selection whose
  // read failed terminally — the holes in the composited picture. Shared by
  // tilesReady()'s release gate and the residency budget's eviction pass so the
  // "keep the only usable coverage over a hole" rule is stated once.
  // With no selection (-1) there is no ceiling and no release/eviction level
  // asymmetry to protect against, so the set is empty.
  std::vector<QRectF> rects;
  if(selected_level_ == -1)
    return rects;
  for(const auto& tile : tiles_)
    if(tile->loadFailed() && tile->level() <= selected_level_)
      rects.push_back(tileSceneRect(*tile));
  return rects;
}

bool GggsTileLayer::coversHole(const GggsTile& tile,
                               const std::vector<QRectF>& holes) const
{
  // [camp#194/#195] True if @p tile is FINER than the selection and overlaps a
  // hole — i.e. it is the only usable coverage over a footprint whose
  // selected-or-coarser tile failed to read. Tiles at or below the selection are
  // not hole coverers: they ARE the picture (and the failed tile's own level).
  if(holes.empty() || selected_level_ == -1 || tile.level() <= selected_level_)
    return false;
  const QRectF tile_rect = tileSceneRect(tile);
  return std::any_of(holes.begin(), holes.end(),
                     [&tile_rect](const QRectF& hole)
                     { return hole.intersects(tile_rect); });
}

bool GggsTileLayer::releaseTiles(const std::vector<GggsTile*>& victims)
{
  // [camp#195] The single release path, extracted from tilesReady() and shared
  // with the residency budget's evictIfOverBudget(). GUI thread only, and only
  // with no loader worker running — both callers gate on that, because this
  // mutates tiles the worker iterates.
  //
  // [camp#194] resetPixels()/releaseGL() pairing invariant (gggs_tile.h): a
  // CPU-only clear on a tile that already uploaded its texture leaves a stale
  // texture shadowing any re-load (texture() returns the old one and never
  // consumes the new data_). With NO context yet (hasContext() == false) no tile
  // can have a texture, so the CPU half alone IS the complete release — this is
  // also the headless-test case. But if a context EXISTS and makeCurrent()
  // FAILED, textures may well exist and we cannot free them, so release NOTHING
  // rather than break the pairing: over-retention costs residency until the next
  // successful pass, a broken pairing costs correctness. (The renderer has
  // latched its GL-failed flag in that case anyway, so nothing is rendering.)
  Q_ASSERT(thread() == QThread::currentThread());
  if(victims.empty())
    return false;
  const bool have_context = renderer_.hasContext() && renderer_.makeCurrent();
  if(renderer_.hasContext() && !have_context)
    return false;
  for(GggsTile* tile : victims)
  {
    if(have_context)
      tile->releaseGL();
    tile->resetPixels();
  }
  if(have_context)
    renderer_.doneCurrent();
  return true;
}

void GggsTileLayer::updateStatus()
{
  // [camp#195] THE status composer. Every condition the layer can report is
  // assembled here from live state, so no writer can clobber another's message
  // (tilesReady() previously rewrote the status unconditionally, which would
  // have wiped the residency budget's over-budget report).
  // [camp#181 / ADR-0015] The anchor part — S-98's permanent indication: name the
  // active anchor AND its source while active, and report a selected but unresolved
  // source as unavailable rather than silently applying nothing. Only on a palette
  // that can carry a shoreline; the unanchored default (mode None) stays silent
  // (byte-identical to pre-camp#181).
  //
  // Computed BEFORE the two early exits below, and appended on every path. "No
  // tiles" and "nothing attempted yet" are states in which the anchor is still set,
  // still persisted, and still applied to whatever loads next — the indication does
  // not lapse because the tile set is momentarily empty.
  QString anchor_part;
  if(const marine_colormap::Palette* pal =
       marine_colormap::find_palette(renderer_.colormap());
     pal && marine_colormap::has_shoreline(*pal))
  {
    const std::optional<double> resolved = shoreline_anchor_.value();
    if(resolved)
      anchor_part = QString("shoreline %1 m (%2)")
                      .arg(*resolved, 0, 'f', 2)
                      .arg(ShorelineAnchor::sourceLabel(shoreline_anchor_.activeSource()));
    else if(shoreline_anchor_.mode() != ShorelineAnchor::Source::None)
      anchor_part = QString("shoreline %1 unavailable")
                      .arg(ShorelineAnchor::sourceLabel(shoreline_anchor_.mode()));
  }
  QStringList parts;
  if(tiles_.empty())
  {
    parts << "no tiles";
    if(!anchor_part.isEmpty())
      parts << anchor_part;
    setStatus("(" + parts.join("; ") + ")");
    return;
  }
  if(!load_started_)
  {
    // Nothing attempted yet — not "no data". Any anchor part still stands.
    setStatus(anchor_part.isEmpty() ? QString() : "(" + anchor_part + ")");
    return;
  }
  // [camp#195] "loading..." is a PART, not an early return. During a pan the
  // loader is re-kicked at every step, so loading_ is true nearly all the time —
  // exactly when the residency reports below matter most. An early return here
  // would hide them until the operator stopped moving.
  if(loading_)
    parts << "loading...";
  // [camp#102] A crossed range after the fold means every loaded tile was
  // all-NoData (or failed to read): there is nothing to draw, and a clear status
  // would leave a silently-blank enabled layer. Not reportable mid-load, where
  // a crossed range only means "nothing has folded yet".
  if(!loading_ && data_min_ > data_max_)
    parts << "no data";
  // [camp#194] Terminally-unreadable tiles. Counted over the WHOLE tile-set (not
  // the visible/selected set) so the number does not flicker with the viewport.
  int failed = 0;
  for(const auto& tile : tiles_)
    if(tile->loadFailed())
      ++failed;
  if(failed > 0)
    parts << QString("%1 tile(s) failed to load").arg(failed);
  // [camp#195] "Report the degraded state; never fail silently" (the issue's
  // ask 3; uma-ADR-0013 D4 puts the same requirement as "the display degrades
  // visibly and predictably rather than churning").
  // The cap is FLOORED at the current-frame working set, so when that
  // set alone exceeds the byte budget the layer does not evict what it is
  // drawing (that is D4's "thrashes by construction") — it exceeds the budget
  // and says so. Relaxing the quality target under this pressure — D4's tau
  // lever — is camp#197.
  if(over_budget_)
    parts << "over the tile budget - raise GggsTileLayers/max_resident_bytes or zoom in";
  // [camp#195] An evicted camp#194 hole coverer cannot come back on its own:
  // loadTilesWorker() skips levels finer than the selection, so the
  // demand-driven reload that makes every other eviction free does not apply to
  // this one class. Surface the loss with the recovery actions that ACTUALLY
  // work: zooming in re-selects the finer level so the loader will read it
  // again, and Rescan helps only by repairing the failed tile itself (which
  // closes the hole a different way).
  // Gated on `failed > 0` so it is a live statement, not a latch: once no tile
  // is failed there is no hole, and the message is moot. rescan() clears the
  // flag outright for the operator's explicit retry.
  if(hole_coverage_released_ && failed > 0)
    parts << "coverage over failed tile(s) released - zoom in or Rescan";
  // [camp#195] The budget cannot be enforced at all — see evictIfOverBudget().
  if(eviction_blocked_)
    parts << "tile budget NOT enforced (GL context unavailable)";
  if(!anchor_part.isEmpty())
    parts << anchor_part;
  setStatus(parts.isEmpty() ? QString() : "(" + parts.join("; ") + ")");
}

std::size_t GggsTileLayer::residentTileCount() const
{
  std::size_t count = 0;
  for(const auto& tile : tiles_)
    if(tile->pixelsLoaded())
      ++count;
  return count;
}

std::size_t GggsTileLayer::perTileResidentBytes() const
{
  // [camp#195] Observed, never assumed (see the header). The largest tile in the
  // set speaks so a non-uniform store cannot undercount the budget.
  // Memoized against tiles_.size(): tile extents are immutable after their
  // metadata read and tiles_ only ever GROWS (loadDirectory/rescan push_back),
  // so the size is a sufficient cache key — and this is called from paint().
  if(per_tile_bytes_count_ == tiles_.size())
    return per_tile_bytes_;
  std::size_t max_pixels = 0;
  for(const auto& tile : tiles_)
    if(tile->valid())
      max_pixels = std::max(max_pixels, std::size_t(tile->width()) *
                                          std::size_t(tile->height()));
  // CPU Float32 buffer + R32F texture of the same extent (gggs_tile.cpp: the CPU
  // copy is deliberately retained past the GPU upload for camp#180). A tile that
  // loaded but never painted holds only the CPU half, so this is deliberately
  // conservative — it charges the fully-displayed cost.
  per_tile_bytes_ = (max_pixels == 0) ? 0 : max_pixels * sizeof(float) * 2;
  per_tile_bytes_count_ = tiles_.size();
  return per_tile_bytes_;
}

std::size_t GggsTileLayer::budgetTiles() const
{
  const std::size_t per_tile = perTileResidentBytes();
  if(resident_budget_bytes_ == 0 || per_tile == 0)
    return 0;   // disabled, or nothing to measure yet
  return std::max<std::size_t>(1, resident_budget_bytes_ / per_tile);
}

std::size_t GggsTileLayer::refreshProtection()
{
  // [camp#195 / uma-ADR-0013 D4] Re-derive the frame's protected working set.
  // The predicate is the LOADER's, not the draw list's — see the header.
  Q_ASSERT(thread() == QThread::currentThread());
  residency_.sync(tiles_.size());
  tile_last_visible_gen_.resize(tiles_.size(), 0);
  residency_.beginFrame();
  ++paint_generation_;

  const std::vector<QRectF> holes = failedFootprints();
  for(std::size_t i = 0; i < tiles_.size(); ++i)
  {
    const GggsTile& tile = *tiles_[i];
    // A terminally-failed tile never becomes resident, so protecting it would
    // inflate the cap floor with memory nothing will ever occupy.
    if(tile.loadFailed())
      continue;
    if(!load_viewport_.isNull() &&
       !tileSceneRect(tile).intersects(load_viewport_))
      continue;
    // Levels finer than the selection are not in the LOADER's set — but two
    // classes of in-view finer tile are nonetheless part of what this frame
    // DRAWS, and neither can reload if dropped (loadTilesWorker skips
    // level > selection), so both are protected:
    //  - any finer tile that is already RESIDENT: itemsIntersecting() draws the
    //    whole resident set with no level filter, so during a zoom-out these
    //    tiles are the entire visible picture until the coarser selection
    //    finishes loading (the camp#103/#194 no-blank-frame guarantee).
    //    tilesReady()'s level-switch release drops them at the right moment, so
    //    protecting them here leaks nothing;
    //  - a camp#194 hole coverer, the only usable coverage over a footprint
    //    whose selected-or-coarser tile failed to read.
    if(selected_level_ != -1 && tile.level() > selected_level_ &&
       !tile.pixelsLoaded() && !coversHole(tile, holes))
      continue;
    residency_.protect(i);
    tile_last_visible_gen_[i] = paint_generation_;
  }
  return residency_.protectedCount();
}

void GggsTileLayer::scheduleEvictionIfNeeded(std::size_t protected_count)
{
  // [camp#195] paint()'s half: publish the over-budget state and QUEUE the
  // eviction. Nothing is released here — releasing a tile inside paint() would
  // mutate the set the render pass reads.
  const std::size_t budget = budgetTiles();
  const bool over = (budget != 0 && protected_count > budget);
  if(over != over_budget_)
  {
    over_budget_ = over;
    updateStatus();
  }
  if(budget == 0 || eviction_pending_)
    return;
  // The cap is floored at the working set (D4), so a frame whose own selection
  // exceeds the budget schedules nothing — there is nothing evictable to gain.
  if(residentTileCount() <= std::max(budget, protected_count))
    return;
  eviction_pending_ = true;
  // Pointer-to-member overload, not the string form: a rename of the slot would
  // otherwise fail at RUNTIME with a qWarning nobody reads, and the residency
  // bound would silently stop existing.
  QMetaObject::invokeMethod(this, &GggsTileLayer::evictIfOverBudget,
                            Qt::QueuedConnection);
}

void GggsTileLayer::evictIfOverBudget()
{
  // [camp#195 / uma-ADR-0013 D4] The eviction pass. GUI thread, off the event
  // loop (or called directly by a headless test through
  // refreshResidencyForTest()).
  Q_ASSERT(thread() == QThread::currentThread());
  eviction_pending_ = false;
  if(budgetTiles() == 0)
  {
    // Budget disabled (or no measurable tile yet). Clear any state the budget
    // path had published so the layer cannot keep advertising a bound it is no
    // longer enforcing.
    if(over_budget_ || eviction_blocked_)
    {
      over_budget_ = false;
      eviction_blocked_ = false;
      updateStatus();
    }
    return;
  }

  // This mutates tiles the loader worker iterates, so it must not run
  // concurrently with it. RE-ARM rather than drop the request: a continuous pan
  // re-kicks the loader every step, so a dropped request would mean the budget
  // almost never fires in exactly the scenario it exists for. (Abort+join, what
  // loadTiles() does, is rejected: a GUI-thread stall for a whole tile read,
  // mid-pan.)
  if(future_watcher_.isRunning())
  {
    eviction_pending_ = true;
    QTimer::singleShot(kEvictionRetryMs, this, &GggsTileLayer::evictIfOverBudget);
    return;
  }

  // Re-derive the protected set LIVE (the MapTiles/camp#98 rule): a tile that
  // re-entered the view between scheduling and now must never be evicted, and
  // the protection recorded at schedule time may be several pan steps stale.
  const std::size_t protected_count = refreshProtection();
  const std::size_t budget = budgetTiles();
  over_budget_ = protected_count > budget;
  const std::size_t cap = std::max(budget, protected_count);
  const std::size_t resident = residentTileCount();
  if(resident <= cap)
  {
    updateStatus();
    return;
  }

  if(!eviction_warned_)
  {
    qWarning().noquote() << "[gggs" << directory_ << "] resident tile budget"
                         << "exceeded (" << qulonglong(resident) << ">"
                         << qulonglong(cap) << "tiles,"
                         << qulonglong(perTileResidentBytes())
                         << "bytes each) - evicting off-viewport tiles";
    eviction_warned_ = true;
  }

  // Evict down to the hysteresis target so the next frame cannot immediately
  // re-trigger (the kReloadHysteresisFactor analogue), but never below the
  // protected working set. With protected_count == 0 (the viewport is entirely
  // off this store's footprint) and a cap of 1, the target truncates to 0 and
  // the whole layer is released — correct: nothing of it is on screen.
  const std::size_t target =
    std::max(protected_count, std::size_t(kEvictHysteresisFactor * double(cap)));

  const std::vector<QRectF> holes = failedFootprints();
  const bool have_centre = !load_viewport_.isNull();
  const QPointF centre = load_viewport_.center();
  const bool have_ladder = available_levels_.size() > 1;
  const int coarsest_level = have_ladder ? available_levels_.front() : -1;

  struct Candidate
  {
    std::size_t index = 0;
    int tier = 0;         // 0 = ordinary, 1 = last resort
    int recent = 0;       // 0 = stale (evict first), 1 = seen recently
    double distance = 0;  // squared, from the viewport centre
    quint64 generation = 0;
  };
  std::vector<Candidate> candidates;
  std::vector<Candidate> coarsest_tiles;
  candidates.reserve(tiles_.size());
  // residency_.candidates() is the evictable partition ONLY — the current
  // frame's protected set is structurally unreachable from here (D4).
  for(const std::size_t index : residency_.candidates())
  {
    const GggsTile& tile = *tiles_[index];
    if(!tile.pixelsLoaded())
      continue;   // nothing resident to free
    Candidate candidate;
    candidate.index = index;
    candidate.generation = tile_last_visible_gen_[index];
    candidate.distance =
      have_centre ? rectDistanceSquared(tileSceneRect(tile), centre) : 0.0;
    // Generation 0 = never protected by any frame. That must classify as STALE:
    // without the != 0 guard it reads as "recent" for the first
    // kRecentGenerations paints of a session, inverting the very ordering the
    // staleness key exists to provide.
    candidate.recent =
      (candidate.generation != 0 &&
       (paint_generation_ - candidate.generation) <= kRecentGenerations) ? 1 : 0;
    // An out-of-view camp#194 hole coverer is a LAST-RESORT candidate: unlike
    // every other tile it cannot reload on pan-back (loadTilesWorker skips
    // levels finer than the selection), so it is dropped only when nothing else
    // can be, and its loss is reported (updateStatus).
    candidate.tier = coversHole(tile, holes) ? 1 : 0;
    if(have_ladder && tile.level() == coarsest_level)
      coarsest_tiles.push_back(candidate);
    else
      candidates.push_back(candidate);
  }

  // The zoom-out floor, BOUNDED: the nearest few coarsest-level tiles are
  // exempt so a zoom-out always has something to draw; the surplus — which is
  // what grows with the area panned — joins the last-resort tier rather than
  // accumulating forever. On a single-level store there is no ladder and hence
  // no exemption, so the budget is not a no-op there.
  //
  // The bound is relative to the CAP as well as absolute. A flat
  // kCoarsestExemptCap can exceed the hysteresis target on a small budget (64
  // exempt tiles against a ~57-tile target at the 512 MiB / 960x960 default),
  // and since exempt tiles still count toward `resident` the victim loop could
  // then never reach the target — it would evict every ordinary candidate on
  // every pass, including tiles the next pan step immediately re-reads. Capping
  // the exemption at a quarter of the cap keeps the floor a floor rather than a
  // second, unevictable budget.
  const std::size_t coarsest_exempt =
    std::min(kCoarsestExemptCap, std::max<std::size_t>(1, cap / 4));
  if(coarsest_tiles.size() > coarsest_exempt)
  {
    std::sort(coarsest_tiles.begin(), coarsest_tiles.end(),
              [](const Candidate& a, const Candidate& b)
              {
                if(a.distance != b.distance)
                  return a.distance < b.distance;   // nearest kept
                return a.generation > b.generation; // then freshest kept
              });
    for(std::size_t k = coarsest_exempt; k < coarsest_tiles.size(); ++k)
    {
      Candidate surplus = coarsest_tiles[k];
      surplus.tier = 1;
      candidates.push_back(surplus);
    }
  }

  // Hybrid ordering (uma-ADR-0013 D4): last-resort tier last; then STALENESS,
  // so a tile not seen for a while goes before one just traversed (D4's
  // objection to distance-only is that it "discards history along a path being
  // traversed"); then farthest-from-viewport first; then LRU generation.
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b)
            {
              if(a.tier != b.tier) return a.tier < b.tier;
              if(a.recent != b.recent) return a.recent < b.recent;
              if(a.distance != b.distance) return a.distance > b.distance;
              return a.generation < b.generation;
            });

  std::vector<GggsTile*> victims;
  bool released_hole_coverage = false;
  for(const Candidate& candidate : candidates)
  {
    if(resident - victims.size() <= target)
      break;
    GggsTile* tile = tiles_[candidate.index].get();
    victims.push_back(tile);
    if(coversHole(*tile, holes))
      released_hole_coverage = true;
  }

  // The auto-range is deliberately NOT recomputed here: the fold is
  // widening-only (foldDataRange), so re-deriving it from a shrinking resident
  // set would make the colormap flicker as tiles come and go. An evicted tile's
  // contribution stays true — its pixels are unchanged on disk, which is exactly
  // what distinguishes eviction from rescan()'s file-replacement case.
  if(releaseTiles(victims))
  {
    eviction_blocked_ = false;
    if(released_hole_coverage)
      hole_coverage_released_ = true;
    cached_image_ = QImage();
    update(boundingRect());
  }
  else if(!victims.empty())
  {
    // releaseTiles() refused: a GL context exists but would not become current.
    // RasterGlRenderer latches that failure and never destroys the context, so
    // this can be PERMANENT — every later pass would release nothing and the
    // residency bound would quietly cease to exist, in a session that is
    // already degraded. Report it instead of growing silently toward the
    // camp#153 OOM.
    eviction_blocked_ = true;
  }
  updateStatus();
}

void GggsTileLayer::tilesReady()
{
  // [camp#102] GUI thread, after the worker's join. Fold the freshly-loaded
  // tiles' ranges into the layer auto-range INCREMENTALLY (widening only — see
  // foldDataRange(): a tile that already contributed keeps its contribution
  // across a level switch, deliberately).
  foldDataRange(false);
  // [camp#103/#194] Once the composited picture's visible set has fully
  // loaded (hasUnloadedVisibleTiles tests every level <= the selection —
  // the same ceiling the loader uses), release the tiles at levels FINER
  // than the selection: they are the zoom-OUT transition backdrop, kept
  // drawing on top (see itemsIntersecting) until the coarser selection's
  // visible tiles are complete — the zoom-out mirror of camp#103's
  // field-verified zoom-in timing, so a LEVEL SWITCH never blanks a region
  // that has coverage at both levels. (The coarse-zoom blank of a region
  // whose only native level is finer than the new selection is a separate,
  // documented limitation of the residency rule — ADR-0013 "Render".)
  // Levels <= the selection are never released: under multi-level
  // compositing they are a permanent part of the picture, not a transient
  // backdrop. Only when no worker is running: this mutates tiles the worker
  // iterates, and a re-kick may already be in flight; the release then
  // happens at that load's own tilesReady().
  // The safety of the mutation rests on the GUI-thread-only invariant (both
  // this slot and every loadTiles() caller) — assert it.
  Q_ASSERT(thread() == QThread::currentThread());
  if(selected_level_ != -1 && !future_watcher_.isRunning() &&
     !hasUnloadedVisibleTiles(load_viewport_))
  {
    // [camp#194 review] The gate above is satisfied by a FAILED tile as well as
    // a loaded one (hasUnloadedVisibleTiles() excludes loadFailed() tiles so the
    // loader can settle — see its contract). "Complete" therefore does NOT imply
    // "covered": where a selected-or-coarser tile failed to read, the picture has
    // a hole, and the resident finer tiles over that footprint are the ONLY
    // usable coverage there. Releasing them would blank a previously-visible
    // region on zoom-out and leave nothing but the failure status — in a nested
    // store one transient coarse RasterIO error (NFS hiccup, a producer swapping
    // the file) throws away good fine data.
    //
    // So release only the footprint that successfully-loaded selected-or-coarser
    // tiles actually cover: a finer tile intersecting ANY failed tile at a level
    // <= the selection is retained. Deliberately conservative — a finer tile that
    // a second, readable coarse tile also covers is kept too. Over-retention costs
    // a little residency until the next successful pass (the failure clears via
    // rescan()'s changed-file refresh or a band switch); under-retention costs
    // the operator their data.
    // NOT a coverage/eviction policy: a region with NO coarse tile at all still
    // releases and blanks per the documented residency rule (ADR-0013 "Render").
    // [camp#195] That is the COVERAGE half of the old camp#195 family and is
    // still unaddressed; camp#195 settled only the residency half (the budget
    // below — ADR-0014), which bounds what is kept and never decides to load
    // finer-than-selection data.
    const std::vector<QRectF> failed_rects = failedFootprints();
    std::vector<GggsTile*> victims;
    for(auto& tile : tiles_)
    {
      // NOTE: this comparison carries NO -1 guard of its own, unlike every
      // other selected_level_ site — it relies entirely on the enclosing
      // `selected_level_ != -1` gate. With -1 (no selection) every tile would
      // satisfy `level() > -1` and the whole resident store would be released.
      // Do not relax the outer gate without adding the guard here. (Since
      // camp#194 loadDirectory()/rescan() also reject tiles whose level parses
      // to -1, so a real tile can never carry the sentinel.)
      if(tile->level() <= selected_level_ || !tile->pixelsLoaded())
        continue;
      // [camp#194 review] Keep this finer tile if it is the only coverage over a
      // footprint whose selected-or-coarser tile failed to read (see above).
      // [camp#195] The hole predicate is shared with the residency budget's
      // eviction pass (coversHole()) so the two retention rules cannot drift.
      if(coversHole(*tile, failed_rects))
        continue;
      victims.push_back(tile.get());
    }
    releaseTiles(victims);
  }
  cached_image_ = QImage();   // re-render now that pixels (and the range) exist
  // [camp#195] The load has settled; every status condition (failed tiles, no
  // data, over budget, released hole coverage) is composed in ONE place —
  // updateStatus(). This slot used to write setStatus() unconditionally, which
  // would silently wipe an over-budget message written from paint().
  loading_ = false;
  updateStatus();
  // [camp#195] Drain a pending residency pass HERE, the one deterministic
  // rendezvous with the loader. The queued/timer pass bails whenever the worker
  // is running, and during a continuous pan paint() re-kicks the loader as soon
  // as it goes idle — so the idle window is about one event-loop turn inside a
  // duty cycle dominated by tile reads, and a 100 ms resample lands in it only
  // by luck. This slot runs on the GUI thread immediately after the worker's
  // join and before any paint() can re-kick, so the pass always gets its turn.
  // The timer stays as a backstop for the case where nothing completes.
  if(eviction_pending_)
    evictIfOverBudget();
  // [camp#142] Keep the Auto resolved range current with the freshly-folded
  // extents (a no-op while the operator holds a Manual override, so an incoming
  // tile never disturbs a pinned range). camp#138 coordination: this is the
  // update_auto() call site to review at merge — the override is applied at
  // render time and is transparent to this fold.
  if(data_min_ <= data_max_)
    range_model_.update_auto(float(data_min_), float(data_max_));
  update(boundingRect());
}

void GggsTileLayer::waitForLoad()
{
  // [camp#102] Test/headless seam: kick the load if it hasn't started, then join
  // and run the ready-fold so renderImage() sees loaded pixels + a valid range.
  // [camp#103] Also re-kick when idle with unloaded visible tiles at the current
  // selection — the headless analogue of paint()'s pan/level re-kick, so a test
  // that changes the LOD via setLodForTest() can drive the load the same way
  // the paint path would. Same moved-since-last-kick guard as paint(): a tile
  // that permanently fails to load must not trigger a redundant kick+join on
  // every call with an unchanged filter.
  if(!load_started_ ||
     (!future_watcher_.isRunning() &&
      (selected_level_ != last_kick_level_ ||
       load_viewport_ != last_kick_viewport_) &&
      hasUnloadedVisibleTiles(load_viewport_)))
  {
    load_started_ = true;
    loadTiles();
  }
  future_watcher_.waitForFinished();
  tilesReady();
}

QRectF GggsTileLayer::boundingRect() const
{
  // Local coordinates: the item is setPos()'d at scene_bounds_.topLeft(), so its
  // own space spans (0,0)..(width,height) in Web-Mercator metres. paint() and
  // drawImage() work here (small magnitudes) to keep QPainter precise.
  return QRectF(QPointF(0.0, 0.0), scene_bounds_.size());
}

float GggsTileLayer::getElevation(const QGeoCoordinate& location) const
{
  // [camp#180] Depth-at-cursor query. Collect the tiles whose geographic extent
  // contains the point, pair each with its tile level, then sample finest-first
  // so the highest-resolution covering tile wins regardless of the rendered LOD.
  // GggsTile::level() is the cached, parse-once level (no per-move QFileInfo +
  // QRegularExpression) — keeping this hot GUI-thread path a plain extent test.
  const double lat = location.latitude();
  const double lon = location.longitude();

  std::vector<std::pair<int, const GggsTile*>> covering;
  for(const auto& tile : tiles_)
  {
    if(lat < tile->minLat() || lat > tile->maxLat() ||
       lon < tile->minLon() || lon > tile->maxLon())
      continue;
    covering.emplace_back(tile->level(), tile.get());
  }

  // Descending level: the finest (highest-level) covering tile is queried first
  // (inspection > display — camp#103 follow-on). Ties keep their relative order.
  std::stable_sort(covering.begin(), covering.end(),
                   [](const auto& a, const auto& b) { return a.first > b.first; });

  for(const auto& [level, tile] : covering)
  {
    (void)level;
    const float value = tile->sampleAt(lon, lat);
    if(!std::isnan(value))
      return value;
  }
  return std::nanf("");
}

QStringList GggsTileLayer::bands() const
{
  // [camp#134] GggsTile bands are 1-indexed; expose them as "1".."N" for the
  // (per-layer) band picker. The renderer consumes only items()/dataRange().
  QStringList result;
  const int count = bandCount();
  for(int b = 1; b <= count; ++b)
    result << QString::number(b);
  return result;
}

RasterBandMeta GggsTileLayer::metadata(const QString&) const
{
  // [camp#134] NoData is per-tile/per-band and only known after a tile loads, so
  // the authoritative sentinel travels per item (see items()); this layer-level
  // metadata is a placeholder for a future unified band picker.
  return RasterBandMeta{};
}

QPair<float, float> GggsTileLayer::dataRange() const
{
  return {float(data_min_), float(data_max_)};
}

QList<RasterFieldItem> GggsTileLayer::items()
{
  return itemsIntersecting(QRectF());
}

QList<RasterFieldItem> GggsTileLayer::itemsIntersecting(const QRectF& clip_scene)
{
  // [camp#134] Collect the loaded tiles as Scalar items for the shared renderer.
  // Called with the renderer's GL context current (renderImage()), so tile->
  // texture() may lazily upload here. The geo->Web-Mercator warp + tessellation
  // now lives in the renderer, so this only forwards each tile's lat/lon extent +
  // per-tile NoData sentinel.
  // [camp#103] A non-null @p clip_scene keeps only tiles whose Web-Mercator
  // extent intersects it — tested BEFORE texture(), so offscreen tiles are
  // neither uploaded nor drawn. geoToMap is monotonic in both axes, so the
  // SW/NE corners bound the tile's scene rect.
  QList<RasterFieldItem> result;
  result.reserve(int(tiles_.size()));
  auto appendTile = [&](GggsTile& tile)
  {
    // [camp#102] Skip a tile whose pixels haven't loaded yet. pixelsLoaded() is an
    // ACQUIRE load pairing with the worker's RELEASE store in loadPixels(), so once
    // true the data_/texture() reads see the worker's completed writes — no race.
    if(!tile.pixelsLoaded())
      return;
    if(!clip_scene.isNull() && !tileSceneRect(tile).intersects(clip_scene))
      return;
    QOpenGLTexture* texture = tile.texture();
    if(!texture)
      return;
    RasterFieldItem item;
    item.texture = texture;
    item.format = RasterFieldItem::Format::Scalar;
    item.geographic = true;
    item.west = tile.minLon();
    item.east = tile.maxLon();
    item.south = tile.minLat();
    item.north = tile.maxLat();
    // [camp#122] Per-tile NoData: a non-uniform store can carry different NoData
    // per tile/band, so it travels per item rather than as one layer uniform.
    item.has_nodata = tile.hasNoData();
    item.nodata = tile.hasNoData() ? float(tile.noData()) : 0.0f;
    result.push_back(item);
  };
  // [camp#194 / ADR-0013] Multi-level compositing: draw EVERY resident tile,
  // in ascending level order (coarse→fine painter's order), with no
  // render-time level filter. WHICH levels are resident is governed by the
  // loader (only levels <= selected_level_ ever load — the ceiling filter in
  // loadTilesWorker) and by tilesReady()'s release (levels > selected_level_
  // drop once the selection's visible set completes). Painter's order then
  // gives:
  //  - steady state: levels <= selection composite, fine overdrawing coarse
  //    where both exist and coarse filling where fine is absent — a
  //    region-disjoint native ladder (ENC chart store) renders all its
  //    regions at every zoom AT-OR-FINER than each region's native level. A
  //    region whose only native level is finer than the selection does not
  //    load (levels > selection never load — the viewport-bounded tradeoff)
  //    and renders blank at coarser zooms even though sceneBounds() includes
  //    its footprint (the coverage half of the old camp#195 family, still
  //    unaddressed — camp#195 settled residency only, ADR-0014);
  //  - zoom-in: the stale coarser levels back the arriving selected level
  //    (unchanged from camp#103's progressive refinement);
  //  - zoom-out: the still-resident finer tiles draw ABOVE the coarse levels
  //    (ascending puts them last) and back the view until the coarser
  //    selection's visible set finishes loading — no blank frame across a
  //    level switch in either direction. That guarantee is about the
  //    TRANSITION only: a region with no coverage at-or-coarser than the
  //    selection still blanks, per the residency rule above (ADR-0013
  //    "Render" coarse-zoom limitation).
  // selected_level_ == -1 (headless, no selection) is the same pass: with no
  // ceiling anywhere, everything loads and everything draws.
  for(const int level : available_levels_)
    for(auto& tile : tiles_)
      if(tile->level() == level)
        appendTile(*tile);
  return result;
}

QImage GggsTileLayer::renderImage(const QSize& size)
{
  return renderImage(size, scene_bounds_);
}

QImage GggsTileLayer::renderImage(const QSize& size, const QRectF& clip_bounds)
{
  if(tiles_.empty() || data_min_ > data_max_ || size.isEmpty() ||
     clip_bounds.isEmpty())
    return QImage();
  // [camp#134] Make the renderer's context current, collect the loaded tiles
  // (uploading their textures under it), then delegate the warp + draw. The
  // returned image is top-down ARGB32 premultiplied (as before).
  if(!renderer_.makeCurrent())
    return QImage();
  const QList<RasterFieldItem> draw = itemsIntersecting(clip_bounds);
  // [camp#181 / ADR-0015] Push the resolved shoreline anchor (manual today) into
  // the renderer before the draw. It bites only on a palette with a
  // shoreline_position; on any other ramp marine_colormap's anchored bake falls
  // back to the plain bake, so this is safe unconditionally. An unchanged value
  // hits the LUT cache.
  const std::optional<double> anchor = shoreline_anchor_.value();
  renderer_.setShorelineAnchor(
    anchor ? std::optional<float>(static_cast<float>(*anchor)) : std::nullopt);
  // [camp#142] Feed the resolved range (Auto tracks data_min_/data_max_; Manual is
  // the operator override) into the shader's u_min/u_max instead of the raw extents.
  const QImage image = renderer_.renderToImage(draw, clip_bounds, range_model_.lo(),
                                               range_model_.hi(), size);
  renderer_.doneCurrent();
  return image;
}

void GggsTileLayer::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*)
{
  if(tiles_.empty())
    return;

  // [camp#103 / ADR-0011] Derive the viewport clip FIRST: the LOD selection and
  // the demand-driven load filter both come from it, so even the very first
  // lazy kick below already loads only the visible tiles at the right level.
  const ViewportClip clip =
    deriveViewportClip(painter, this, boundingRect(), scene_bounds_, kMaxImageEdge);

  // [camp#103 / ADR-0013] Level-by-view-scale. clip.scene is Web-Mercator, whose
  // metres are inflated by ~sec(latitude) vs true ground metres; fromCellSize
  // expects ground metres, so convert at the viewport centre (metersPerUnit =
  // cos(lat)) — at 43°N the difference is ~1.37×, about half a level.
  bool level_changed = false;
  if(!available_levels_.empty() && clip.size.width() > 0)
  {
    const double mercator_mpp = clip.scene.width() / double(clip.size.width());
    const double ground_mpp =
      mercator_mpp * web_mercator::metersPerUnit(clip.scene.center());
    const int target = selectLodLevel(ground_mpp, available_levels_);
    level_changed = (target != selected_level_);
    if(level_changed)
    {
      // [camp#103 field verify / camp#194] Do NOT release the outgoing level
      // here. On zoom-in the coarser levels stay a permanent part of the
      // composited picture; on zoom-out the finer levels keep rendering
      // (above the coarse — see itemsIntersecting) as the transition
      // backdrop until tilesReady() releases them once the new selection's
      // visible tiles finish loading. The original eager release blanked the
      // layer for the whole load on every zoom across a level boundary —
      // very visible flicker.
      selected_level_ = target;
      cached_image_ = QImage();
    }
  }

  // Snapshot the load viewport BEFORE any kick so the worker's filter can never
  // read stale bounds (review must-fix), then kick when needed:
  //  - first paint: the one-time lazy kick (now already level+viewport filtered);
  //  - level change: reload at the new level immediately (abort+join above);
  //  - idle + a pan/zoom exposed unloaded visible tiles at the selected level
  //    AND the filter actually moved since the last kick (review must-fix: a
  //    pure pan must re-kick or panned-in regions stay blank forever; the
  //    moved-since-last-kick guard keeps a permanently-failing tile from
  //    re-kicking every frame, and kicking only when idle avoids the
  //    abort+join stall a pan storm would otherwise pay per frame).
  load_viewport_ = clip.scene;

  // [camp#195 / uma-ADR-0013 D4] Protect this frame's working set and schedule
  // the residency pass. Placement is load-bearing on both sides:
  //  - AFTER load_viewport_ is set, because the protection predicate IS the
  //    loader's filter (level <= selection, intersecting the load viewport);
  //  - BEFORE every remaining early return (the crossed-range return below),
  //    because refreshProtection() begins by splicing the whole protected
  //    partition back into the evictable one — returning between beginFrame()
  //    and protect() would leave the live visible set evictable.
  // The tiles_.empty() return at the top of paint() is vacuous here: with no
  // tiles there is nothing to protect and nothing to evict.
  scheduleEvictionIfNeeded(refreshProtection());

  if(!load_started_)
  {
    load_started_ = true;
    loadTiles();
  }
  else if(level_changed)
    loadTiles();
  else if(!future_watcher_.isRunning() &&
          (selected_level_ != last_kick_level_ ||
           load_viewport_ != last_kick_viewport_) &&
          hasUnloadedVisibleTiles(clip.scene))
    loadTiles();

  if(data_min_ > data_max_)   // no tile's pixels/range folded yet
    return;

  // [camp#103 / ADR-0011] Render only the viewport-visible clip of the extent,
  // sized to its on-screen pixels — a zoomed-in view of a store far larger than
  // kMaxImageEdge stays crisp. Re-render when the size (zoom) OR the clip (pan)
  // changes; a viewport-sized FBO makes the per-frame pan re-render cheap.
  if(cached_image_.isNull() || cached_size_ != clip.size ||
     cached_clip_ != clip.scene)
  {
    cached_image_ = renderImage(clip.size, clip.scene);
    cached_size_ = clip.size;
    cached_clip_ = clip.scene;
  }
  if(cached_image_.isNull())
    return;

  painter->save();
  // [camp#132] Smoothing is operator-opt-in per layer; the default Nearest blit
  // keeps data cells faithful for QA (the GL data filter is Nearest regardless).
  painter->setRenderHint(QPainter::SmoothPixmapTransform, smooth_interpolation_);
  painter->drawImage(clip.local, cached_image_);
  painter->restore();
}

void GggsTileLayer::setSmoothInterpolation(bool smooth)
{
  if(smooth == smooth_interpolation_)
    return;
  smooth_interpolation_ = smooth;
  writeSettings();
  update(boundingRect());   // blit-hint-only change: no re-render needed
}

void GggsTileLayer::setColormap(const std::string& name)
{
  if(name == renderer_.colormap())
    return;
  renderer_.setColormap(name);   // re-bakes the LUT on next render
  cached_image_ = QImage();      // force a re-render with the new ramp
  writeSettings();
  // [camp#181 / ADR-0015] The anchor part of the status is gated on whether the
  // NEW palette carries a shoreline, so a palette switch can start or stop
  // anchoring. Recompose here or the status is stale: switching onto oleron would
  // show no anchor report at all, and switching off it would leave the previous
  // one asserting an anchor that no longer bites.
  updateStatus();
  update(boundingRect());
}

void GggsTileLayer::setRangeOverride(float lo, float hi)
{
  // [camp#142] Pin the resolved range to the operator's [lo, hi] (set_manual swaps
  // an inverted pair so lo() <= hi()). Re-render with the new range + persist.
  range_model_.set_manual(lo, hi);
  cached_image_ = QImage();
  writeSettings();
  update(boundingRect());
}

void GggsTileLayer::resetRangeToAuto()
{
  // [camp#142] Return to data-driven Auto, then immediately re-track the current
  // extents so lo()/hi() reflect the data without waiting for the next fold.
  range_model_.reset();
  if(data_min_ <= data_max_)
    range_model_.update_auto(float(data_min_), float(data_max_));
  cached_image_ = QImage();
  writeSettings();
  update(boundingRect());
}

bool GggsTileLayer::paletteSupportsAnchor() const
{
  const marine_colormap::Palette* pal =
    marine_colormap::find_palette(renderer_.colormap());
  return pal && marine_colormap::has_shoreline(*pal);
}

void GggsTileLayer::applyShorelineAnchor(ShorelineAnchor::Source mode,
                                         std::optional<double> manual)
{
  // [camp#181 / ADR-0015] Called only from a real operator change — the dialog
  // seeds itself from state.anchor_mode and does NOT fire on open, so opening it
  // cannot write anything back. The guard below stays as cheap insurance for any
  // future caller that re-sends an unchanged pair. The holder emits changed()
  // (→ cache drop + status + repaint, wired in the ctor) only when the resolved
  // anchor moves, so setting an unchanged value is free. `manual` is the
  // operator's typed value INDEPENDENT of the mode, so selecting None keeps it
  // for a later switch back rather than discarding it.
  if(mode == shoreline_anchor_.mode() && manual == shoreline_anchor_.manualValue())
    return;
  shoreline_anchor_.applyManualSelection(manual, mode);
  writeSettings();
}

int GggsTileLayer::bandCount() const
{
  // [camp#108] The tiles of a store are uniform, so the first valid tile's band
  // count speaks for the layer. loadDirectory()/rescan() only keep valid() tiles,
  // so front() is valid when non-empty.
  return tiles_.empty() ? 0 : tiles_.front()->bandCount();
}

std::vector<int> GggsTileLayer::tileBands() const
{
  // [camp#108] Test-only seam (see header): report each tile's current band so a
  // headless test can assert the per-tile band propagation that the rendered image
  // cannot isolate — e.g. that a rescan after a band switch added the new tile on
  // the layer's band, not the fresh-tile default of 1.
  std::vector<int> bands;
  bands.reserve(tiles_.size());
  for(const auto& tile : tiles_)
    bands.push_back(tile->band());
  return bands;
}

void GggsTileLayer::setBand(int band)
{
  // [camp#108] Persisting public entry point. Validate + apply the band switch
  // (the actual work lives in applyBand()), then round-trip the selection to
  // QSettings. The guard mirrors applyBand()'s so a no-op / out-of-range pick
  // doesn't pay a needless settings write (the colormap path guards the same way).
  if(band < 1 || band > bandCount() || band == band_)
    return;
  applyBand(band);
  writeSettings();
}

void GggsTileLayer::applyBand(int band)
{
  // [camp#108] Band switch WITHOUT persisting — the shared body of setBand()
  // (which persists after) and readSettings() (which applies the already-persisted
  // value, so must NOT write it back). Decoupling the read path from a settings
  // write mirrors the inline colormap apply in readSettings().
  //
  // Validate against the tile-set's band count and skip the no-change path (so a
  // persisted-band read or a re-click of the current band doesn't pay the abort +
  // reload). bandCount() == 0 (no tiles) rejects everything.
  if(band < 1 || band > bandCount() || band == band_)
    return;
  band_ = band;

  // Abort + join any in-flight load BEFORE mutating the tiles the worker reads —
  // same contract as loadTiles()/rescan() (the worker captures `this` and iterates
  // tiles_). Whole-tile abort granularity, as elsewhere.
  if(future_watcher_.isRunning())
  {
    abort_flag_mutex_.lock();
    abort_flag_ = true;
    abort_flag_mutex_.unlock();
    future_watcher_.waitForFinished();
  }

  // Re-point each tile that carries the requested band at the new band: release
  // its old-band GL texture under this layer's own context (so the next render
  // re-uploads the new band's pixels) then setBand() (clears its CPU pixels +
  // range, marks it not-loaded). releaseGL() touches textures only, NOT the
  // shader/FBO/LUT, which are band-independent. The context handling (null context
  // vs makeCurrent failure) is guarded just below.
  //
  // [camp#108] bandCount() speaks for the layer via tiles_.front(); a tile with
  // fewer bands than the front (a non-uniform tile-set, e.g. mixed survey dirs)
  // can't serve the requested band. Such tiles lacking the requested band keep
  // their prior band — we do NOT release their texture or clear their pixels, so
  // they keep rendering normally on the band they already hold — and are excluded
  // from the range fold (tilesReady() skips tiles whose band() != band_) so their
  // stale prior-band range can't pollute the new band's auto-range. WARN once per
  // switch so the drop isn't invisible.
  // A null/not-yet-created context (the layer never painted — applyBand() can fire
  // from readSettings() before first paint) is expected: no textures exist yet, so
  // the switch just clears pixels + reloads with no stale GL state — skip the
  // release pass entirely (hasContext() == false) rather than forcing the offscreen
  // context into existence early. If the context EXISTS but makeCurrent FAILS, the
  // renderer latches its GL-failed flag (renderImage() then returns null and the
  // layer stops rendering rather than drawing a stale-band frame) and we leave the
  // textures alone.
  bool have_context = renderer_.hasContext() && renderer_.makeCurrent();
  int dropped = 0;
  for(auto& tile : tiles_)
  {
    if(tile->bandCount() < band)
    {
      ++dropped;
      continue;   // leave its texture/pixels/range on the prior band
    }
    if(have_context)
      tile->releaseGL();
    tile->setBand(band);
  }
  if(have_context)
    renderer_.doneCurrent();
  if(dropped > 0)
    qWarning("GggsTileLayer: %d tile(s) lack band %d; left on their prior band",
             dropped, band);

  // Reset the layer auto-range to crossed — the new band's range is unknown until
  // its pixels reload. tilesReady() re-folds it (over current-band tiles only)
  // after the load completes.
  data_min_ = 1.0;
  data_max_ = 0.0;
  cached_image_ = QImage();

  // Re-kick the async load only if the layer already started one (first paint).
  // Otherwise the lazy first-paint kick will read the new band; no need to force
  // a load on a layer the operator may never turn on.
  if(load_started_)
    loadTiles();
  update(boundingRect());
}

void GggsTileLayer::contextMenu(QMenu* menu)
{
  map::Layer::contextMenu(menu);

  // [camp#104] Manual refresh affordance. The retired GggsStoreLayer's
  // QFileSystemWatcher (camp#102 live tile/epoch pickup) was dropped with it
  // (ADR-0005), so a flat layer no longer auto-picks-up tiles that land after it
  // loaded. This action is the stopgap: re-enumerate the tile-set directory for
  // newly-landed `*.tif` tiles on demand — right-click → Rescan instead of
  // restarting CAMP. Safe to invoke repeatedly / when nothing changed (rescan()
  // adds only paths not already held and no-ops otherwise). A per-layer watcher
  // (live auto-pickup) remains a follow-up.
  QAction* rescan_action = menu->addAction("Rescan");
  connect(rescan_action, &QAction::triggered, this, [this]() { rescan(); });

  // [camp#132] Per-layer blit-smoothing opt-in (default OFF = Nearest, the
  // faithful-QA baseline; interpolation fabricates values that aren't in the
  // data and can mask the artifacts the operator is looking for).
  QAction* smooth_action = menu->addAction("Smooth interpolation");
  smooth_action->setCheckable(true);
  smooth_action->setChecked(smooth_interpolation_);
  connect(smooth_action, &QAction::triggered, this,
          [this](bool on) { setSmoothInterpolation(on); });

  // [camp#141] Expose the FULL marine_colormap registry (grayscale/bronze/thermal/
  // viridis/turbo/quality), not just the three legacy ramps.
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(const std::string& name : marine_colormap::palette_names())
  {
    QAction* action = colormap_menu->addAction(QString::fromStdString(name));
    action->setCheckable(true);
    action->setChecked(name == renderer_.colormap());
    connect(action, &QAction::triggered, this, [this, name]() { setColormap(name); });
  }

  // [camp#142 PR2] Colormap range override. GGGS tiles are always scalar, so the
  // action is offered unconditionally (same gating as the Colormap submenu above).
  // Opens the interactive colorbar — drag the handles or edit the bounds to pin a
  // Manual override, reset to track the data extents — the successor to PR1's
  // sequential numeric prompts.
  QAction* range_action = menu->addAction("Colormap range…");
  connect(range_action, &QAction::triggered, this, [this]()
  {
    ColormapRangeState state;
    const auto idx = marine_colormap::palette_index(renderer_.colormap());
    state.palette_index = idx ? static_cast<int>(*idx) : 0;
    state.data_min = static_cast<float>(data_min_);
    state.data_max = static_cast<float>(data_max_);
    state.mode = range_model_.mode();
    state.lo = range_model_.lo();
    state.hi = range_model_.hi();
    // [camp#181 / ADR-0015] Anchor state for the dialog's shoreline control.
    state.palette_name = renderer_.colormap();
    state.supports_anchor = paletteSupportsAnchor();
    state.anchor_mode = shoreline_anchor_.mode();
    state.manual_anchor = shoreline_anchor_.manualValue();
    showColormapRangeDialog(
      nullptr, "Colormap range", state,
      [this](float lo, float hi) { setRangeOverride(lo, hi); },
      [this]() { resetRangeToAuto(); },
      [this](ShorelineAnchor::Source mode, std::optional<double> manual)
      { applyShorelineAnchor(mode, manual); });
  });

  // [camp#108] Band picker — only for multi-band tile-sets (bathy depth +
  // uncertainty, backscatter intensity + quality). Single-band stores (the
  // common sidescan case) get no submenu, so no visual noise. One checkable
  // action per 1-indexed band, checked when it is the current selection.
  const int bands = bandCount();
  if(bands > 1)
  {
    QMenu* band_menu = menu->addMenu("Band");
    for(int b = 1; b <= bands; ++b)
    {
      QAction* action = band_menu->addAction(QString::number(b));
      action->setCheckable(true);
      action->setChecked(b == band_);
      connect(action, &QAction::triggered, this, [this, b]() { setBand(b); });
    }
  }
}

QString GggsTileLayer::settingsKey() const
{
  // [camp#126] Identity is the DIRECTORY, not the display name. The base
  // MapItem::settingsKey() returns itemID() (parent path + objectName()), but a
  // store layer's objectName() is a parent/leaf folder label two distinct stores
  // can share (survey_a/bathymetry/processed and survey_b/bathymetry/processed
  // both display as "bathymetry/processed"), so itemID()-keyed persistence would
  // collide them onto ONE QSettings group — one store's visible/colormap/band
  // would overwrite the other's. The absolute directory path is unique and stable,
  // so key on it instead. Percent-encode it (every '/' becomes %2F) so it is a
  // single FLAT key rather than a deep nested group tree, and prefix with "dir:"
  // to keep it readable and namespaced. NO migration: moving off the old name-key
  // accepts a one-time reset of currently-saved prefs (pre-deployment).
  // [camp#126] directory_ is already canonicalized to an absolute path by the
  // ctor, so it can be keyed directly — no QDir::absolutePath() needed here.
  return "dir:" + QString::fromLatin1(QUrl::toPercentEncoding(directory_));
}

void GggsTileLayer::readSettings()
{
  map::Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(settingsKey());
  // [camp#102] Default GGGS tile-set leaves OFF: re-read `visible` with a FALSE
  // fallback (Layer::readSettings just applied it with a TRUE default). This has
  // to live in the leaf override, not a ctor setVisible(false) — MapItem::
  // itemConstructed() runs readSettings() via QTimer::singleShot(0,...) AFTER the
  // ctor, which would clobber a ctor call (map_item.cpp:26,180-183). A persisted
  // `visible` value still wins (the operator's on/off choice round-trips); only
  // the first-run default flips.
  setVisible(settings.value("visible", false).toBool());
  // [camp#141] Persisted palette name. Read case-insensitively (camp already stores
  // lowercase, but tolerate a legacy capitalized "Viridis"/"Turbo") and validate
  // against the marine_colormap registry; an unknown name falls back to grayscale.
  std::string colormap = settings.value(
    "colormap", QString::fromStdString(renderer_.colormap())).toString().toLower().toStdString();
  if(!marine_colormap::palette_index(colormap))
    colormap = "grayscale";
  // [camp#108] Persisted band (default 1). Applied via applyBand() below — the
  // non-persisting band switch (texture release + reload + range reset) — so the
  // read path does NOT write the value straight back out (setBand() would). Only
  // when it differs, to skip the abort+reload on the common no-change path,
  // mirroring the inline colormap apply directly below.
  const int band = settings.value("band", 1).toInt();
  // [camp#142] Persisted colormap range. "manual" restores the operator override;
  // anything else (default "auto") leaves the data-driven Auto range. Honor Manual
  // only when both extents are present too — a partial/corrupt entry falls back to
  // Auto rather than snapping to the [0,1] read-defaults.
  const QString range_mode = settings.value("range_mode", "auto").toString();
  const bool has_manual_range = range_mode == "manual" &&
    settings.contains("range_min") && settings.contains("range_max");
  const float range_min = settings.value("range_min", 0.0).toFloat();
  const float range_max = settings.value("range_max", 1.0).toFloat();
  // [camp#132] Persisted blit-smoothing opt-in (default OFF = Nearest).
  smooth_interpolation_ = settings.value("smooth_interpolation", false).toBool();
  // [camp#181 / ADR-0015] Persisted manual shoreline anchor. Present -> Manual mode
  // at that value; absent -> the unanchored None default (byte-identical to
  // pre-camp#181). Applied below, after the group is closed.
  // The bool*ok overload is load-bearing: a corrupt/unparsable entry makes
  // toDouble() return 0.0, and 0.0 is the one value ADR-0015 D6 forbids. Without
  // the check a garbled key would restore as a Manual anchor at sea level.
  bool anchor_ok = false;
  const double anchor_value =
    settings.value("shoreline_anchor").toDouble(&anchor_ok);
  const bool has_anchor = settings.contains("shoreline_anchor") && anchor_ok;
  // The MODE is persisted explicitly, never inferred from the value's presence.
  // The dialog deliberately KEEPS a typed manual value when the operator selects
  // None (so switching back restores it), so "Manual -28.038 -> None" leaves the
  // value stored with no anchor selected; inferring Manual from it would restore
  // an anchor nobody chose — the same class D6 guards against, arriving through
  // mode inference instead of through 0.0. An absent or unrecognized token (a
  // settings file written before this key existed, or a hand-edited one) restores
  // as None for the same reason: unanchored is the only honest default, and it is
  // also what makes a future ChartDatum/PlatformTide selection restorable at all.
  const ShorelineAnchor::Source anchor_mode =
    ShorelineAnchor::sourceFromKey(
      settings.value("shoreline_anchor_mode").toString())
      .value_or(ShorelineAnchor::Source::None);
  settings.endGroup();
  settings.endGroup();
  if(colormap != renderer_.colormap())
  {
    renderer_.setColormap(colormap);
    cached_image_ = QImage();
  }
  if(band != band_)
    applyBand(band);
  // Apply the persisted range AFTER the band switch (applyBand resets data_min_/
  // data_max_; a Manual override is independent of the data extents and survives).
  if(has_manual_range)
    range_model_.set_manual(range_min, range_max);
  else
    range_model_.reset();
  // [camp#181 / ADR-0015] Restore the persisted (value, mode) pair as ONE change,
  // so no observer sees the value paired with a mode it was not stored with.
  shoreline_anchor_.applyManualSelection(
    has_anchor ? std::optional<double>(anchor_value) : std::nullopt, anchor_mode);
}

void GggsTileLayer::writeSettings()
{
  map::Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(settingsKey());
  settings.setValue("colormap", QString::fromStdString(renderer_.colormap()));
  settings.setValue("band", band_);   // [camp#108] selected band round-trips
  // [camp#142] Persist the colormap range mode + bounds so a Manual override (and
  // its [lo, hi]) survives a restart; Auto persists as "auto".
  settings.setValue("range_mode",
                    range_model_.mode() == marine_colormap::RangeMode::Manual ? "manual"
                                                                              : "auto");
  settings.setValue("range_min", range_model_.lo());
  settings.setValue("range_max", range_model_.hi());
  settings.setValue("smooth_interpolation", smooth_interpolation_);   // [camp#132]
  // [camp#181 / ADR-0015] Persist the shoreline anchor's MODE and its manual value
  // as two independent keys: the mode is what the operator chose, the value is what
  // they typed, and selecting None deliberately KEEPS the typed value for a later
  // switch back — so the value's presence cannot stand in for the mode. Chart datum
  // and platform tide persist as a mode only; their values are resolved live by
  // their sources in later PRs, never saved as a number.
  settings.setValue("shoreline_anchor_mode",
                    ShorelineAnchor::sourceKey(shoreline_anchor_.mode()));
  if(shoreline_anchor_.manualValue())
    settings.setValue("shoreline_anchor", *shoreline_anchor_.manualValue());
  else
    settings.remove("shoreline_anchor");
  settings.endGroup();
  settings.endGroup();
}

void GggsTileLayer::onRemovedFromMap()
{
  // [camp#104] Drop this tile-set directory from the BackgroundManager restore
  // list (GggsTileLayers/dirs) so a user-removed flat layer stays gone next
  // session — the flat-layer analogue of the retired GggsStoreLayer root drop.
  QSettings settings;
  QStringList dirs = settings.value("GggsTileLayers/dirs").toStringList();
  if(dirs.removeAll(directory_) > 0)
    settings.setValue("GggsTileLayers/dirs", dirs);
}

}  // namespace raster
}  // namespace camp
