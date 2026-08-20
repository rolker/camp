#include "sonar_live_cache_layer.h"

#include "../node.h"
#include "../../map_view/web_mercator.h"
#include "../../raster/colormap_range_dialog.h"
#include "../../raster/viewport_clip.h"

#include <marine_colormap/palette.hpp>

#include <QAction>
#include <QDebug>
#include <QDir>
#include <QGeoCoordinate>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMenu>
#include <QMetaObject>
#include <QOpenGLTexture>
#include <QPainter>
#include <QSettings>
#include <QStandardPaths>
#include <QTransform>
#include <QUrl>
#include <QtConcurrent>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

namespace camp
{
namespace ros
{
namespace live_coverage
{

namespace
{

// [camp#134] The duplicated kVertex/kFragment shaders, ensureProgram(), ensureLut()
// and the per-vertex geo->Web-Mercator tessellation moved into the shared
// raster::RasterGlRenderer (see ADR-0007), which also fixes the NaN NoData discard.
// This layer now collects its held tiles into RasterFieldItems and delegates the
// draw; only the data source differs (in-memory dequantized Float32 vs. GDAL read).
// ADR-0006's persistence + opt-in-subscription contract is untouched by this change.

// Flat, filesystem-safe token for a source namespace (percent-encode every '/').
QString sanitize(const std::string& ns)
{
  return QString::fromLatin1(QUrl::toPercentEncoding(QString::fromStdString(ns)));
}

// [camp#160] Cache sub-directory for overview pyramid tiles, kept apart from the
// fine tiles at the cache-dir root so warm-load of fine tiles can't pick them up.
const std::string kOverviewSubdir = "overviews";

// [camp#160] Squared distance (Web-Mercator scene units) from a tile's centre to
// a scene point — the eviction key for view-based LOD (farthest tile evicts first).
double tileSceneDistanceSquared(const SonarLiveTile& tile, const QPointF& point)
{
  const QPointF centre = web_mercator::geoToMap(QGeoCoordinate(
    0.5 * (tile.minLat() + tile.maxLat()), 0.5 * (tile.minLon() + tile.maxLon())));
  const double dx = centre.x() - point.x();
  const double dy = centre.y() - point.y();
  return dx * dx + dy * dy;
}

// [camp#172] Web-Mercator scene rect for a GGGS index, from its geographic extent
// alone (no live tile needed — the tile has been evicted). Matches the extent math
// itemsIntersecting() uses per tile, so the reload viewport test agrees with the draw
// filter. North-up normalized.
QRectF indexSceneRect(const gggs::GridIndex& index)
{
  const QPointF lo = web_mercator::geoToMap(
    QGeoCoordinate(index.southLatitude(), index.westLongitude()));
  const QPointF hi = web_mercator::geoToMap(
    QGeoCoordinate(index.northLatitude(), index.eastLongitude()));
  return QRectF(lo, hi).normalized();
}

// [camp#172] Off-thread reload worker: load each evicted fine tile's cached GeoTIFF
// from the fine-tile cache dir. Takes everything by value so it is fully
// self-contained — safe even if the layer is destroyed before it finishes (same
// contract as writeTileToCache). Skips any index whose file is missing/unreadable;
// the caller clears every attempted index regardless (onReloadFinished).
std::vector<SonarLiveTile> reloadTilesFromCache(std::vector<gggs::GridIndex> indices,
                                                std::string dir, std::uint8_t level_value)
{
  namespace fs = std::filesystem;
  const gggs::Level level(level_value);
  std::vector<SonarLiveTile> loaded;
  loaded.reserve(indices.size());
  for(const gggs::GridIndex& index : indices)
  {
    const std::string stem = std::to_string(static_cast<int>(index.level())) + "_" +
                             std::to_string(index.row()) + "_" +
                             std::to_string(index.column()) + ".tif";
    const std::string path = (fs::path(dir) / stem).string();
    auto tile = SonarLiveTile::loadFromGeoTiff(path, level);
    if(tile && tile->index().valid())
      loaded.push_back(std::move(*tile));
  }
  return loaded;
}

// Off-thread write-through worker. Takes everything by value so it is fully
// self-contained — safe even if the layer is destroyed before it finishes.
// Atomic temp+rename for crash safety only (ADR-0006 D2).
void writeTileToCache(SonarLiveTile tile, std::string dir)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(dir, ec);
  if(ec)
    return;   // unwritable cache dir — skip the GDAL round-trip (best-effort cache)
  const std::string stem = std::to_string(static_cast<int>(tile.index().level())) + "_" +
                           std::to_string(tile.index().row()) + "_" +
                           std::to_string(tile.index().column());
  const std::string final_path = (fs::path(dir) / (stem + ".tif")).string();
  const std::string tmp_path = final_path + ".tmp";
  if(!tile.writeToGeoTiff(tmp_path))
  {
    fs::remove(tmp_path, ec);
    return;
  }
  fs::rename(tmp_path, final_path, ec);
  if(ec)
    fs::remove(tmp_path, ec);
}

}  // namespace

SonarLiveCacheLayer::SonarLiveCacheLayer(MapItem* parent, Node* node,
                                         const QString& base_namespace):
  Layer(parent, node, "Live Coverage [" + base_namespace + "]"),
  base_namespace_(base_namespace.toStdString())
{
  // Cache dir: <base>/<sanitized source ns>, base defaults to AppDataLocation but
  // is operator-overridable (ADR-0006 D2 / #117 no-hardcoded-defaults).
  QSettings settings;
  const QString base_dir = settings.value(
    "LiveTileCache/cache_dir",
    QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) +
      "/live_tile_cache").toString();
  cache_dir_ = QDir(base_dir).filePath(sanitize(base_namespace_)).toStdString();

  // [camp#160] Resident fine-tile footprint budget for eviction. Default 512 MiB,
  // operator-overridable (ADR-0006 D2 / #117: a default, never an un-changeable
  // hardcode). 0 disables eviction (unbounded — the pre-#160 behaviour).
  constexpr qulonglong kDefaultBudgetBytes = 512ull * 1024 * 1024;
  vram_budget_bytes_ = static_cast<std::size_t>(
    settings.value("LiveTileCache/max_vram_bytes", kDefaultBudgetBytes).toULongLong());

  // [camp#172] The on-demand reload worker publishes its loaded tiles on the GUI
  // thread via onReloadFinished() (queued by Qt from the watcher's finished signal).
  connect(&reload_watcher_, &QFutureWatcher<std::vector<SonarLiveTile>>::finished, this,
          [this]() { onReloadFinished(); });

  // Availability is cheap: subscribe to the catalog even while inactive so the
  // operator can see how much coverage the source holds. The tile stream stays
  // unsubscribed until enableLiveCoverage().
  subscribeCatalog();
  updateDisplay();
}

SonarLiveCacheLayer::~SonarLiveCacheLayer()
{
  // Reset the subscriptions FIRST so no executor-thread callback can marshal a new
  // handler onto this object mid-teardown (closes the TOCTOU window; matches the
  // teardown-symmetry of the other ROS layers).
  tile_sub_.reset();
  catalog_sub_.reset();

  // Stop coalesced relaunches, then JOIN every in-flight write worker (not just the
  // latest) so none can outlive the object. Each worker is self-contained, so this
  // is purely a lifetime join. Then tear down GL.
  shutting_down_ = true;
  for(QFutureWatcher<void>* watcher : write_watchers_)
    watcher->waitForFinished();
  // [camp#172] Join the on-demand reload worker too (it is self-contained, so this is
  // a pure lifetime join; shutting_down_ makes its finished-slot a no-op).
  reload_watcher_.waitForFinished();
  // [camp#134] Release each tile's GL texture under the renderer's context (the
  // textures are owned by the Entries, not the renderer). The renderer frees its
  // own program/LUT/FBO/context in its destructor right after this.
  if(renderer_.makeCurrent())
  {
    for(auto& item : tiles_)
      item.second.texture.reset();
    for(auto& item : overview_tiles_)   // [camp#160] overview textures too
      item.second.texture.reset();
    renderer_.doneCurrent();
  }
}

QString SonarLiveCacheLayer::settingsKey() const
{
  // Identity is the source namespace, not the display label (which embeds it but
  // is a human string). Flat percent-encoded key, prefixed for readability.
  return "live:" + sanitize(base_namespace_);
}

// ------------------------------- subscriptions -------------------------------

void SonarLiveCacheLayer::subscribeCatalog()
{
  if(catalog_sub_ || !node_ || !node_->node())
    return;
  // Complete snapshot, latest wins; transient-local so a late joiner gets the
  // current catalog immediately (matches the boat-side producer QoS).
  rclcpp::QoS qos(1);
  qos.transient_local().reliable();
  const std::string topic = base_namespace_ + "/coverage_catalog";
  catalog_sub_ = node_->node()->create_subscription<marine_interfaces::msg::TileCatalog>(
    topic, qos,
    [this](marine_interfaces::msg::TileCatalog::SharedPtr msg)
    {
      // ROS thread: copy + marshal to the GUI thread (ADR-0001). Never touch the
      // reconciler / tiles here.
      QMetaObject::invokeMethod(
        this, [this, m = *msg]() { handleCatalog(m); }, Qt::QueuedConnection);
    });
}

void SonarLiveCacheLayer::subscribeTiles()
{
  if(!node_ || !node_->node())
    return;
  if(!tile_sub_)
  {
    rclcpp::QoS qos(10);
    qos.best_effort();
    const std::string topic = base_namespace_ + "/coverage_tiles";
    tile_sub_ =
      node_->node()->create_subscription<marine_interfaces::msg::SonarVisualizationTile>(
        topic, qos,
        [this](marine_interfaces::msg::SonarVisualizationTile::SharedPtr msg)
        {
          QMetaObject::invokeMethod(
            this, [this, m = *msg]() { handleTile(m); }, Qt::QueuedConnection);
        });
  }
  if(!request_pub_)
  {
    rclcpp::QoS qos(10);
    qos.reliable();
    request_pub_ = node_->node()->create_publisher<marine_interfaces::msg::TileRequest>(
      base_namespace_ + "/coverage_requests", qos);
  }
}

void SonarLiveCacheLayer::unsubscribeTiles()
{
  tile_sub_.reset();
  request_pub_.reset();
}

void SonarLiveCacheLayer::publishRequest(const std::vector<gggs::GridIndex>& tiles)
{
  if(tiles.empty() || !request_pub_ || !node_ || !node_->node())
    return;
  marine_interfaces::msg::TileRequest msg;
  msg.header.stamp = node_->node()->now();
  msg.header.frame_id = "gggs";
  msg.tiles.reserve(tiles.size());
  for(const auto& index : tiles)
    msg.tiles.push_back(tileIndexFromGridIndex(index));
  request_pub_->publish(msg);
}

// -------------------------------- activation ---------------------------------

void SonarLiveCacheLayer::enableLiveCoverage()
{
  if(enabled_)
    return;
  enabled_ = true;
  // Warm-load the disk cache BEFORE subscribing so the in-memory state is seeded
  // (and the reconciler primed) before any live tile/catalog arrives — and so the
  // GUI-thread warm-load can't race a callback (ADR-0006 D3/D4).
  warmLoad();
  subscribeTiles();
  // [camp#169] Replay the buffered catalog so reconcile (and the resulting tile
  // requests) fire on every enable — the latched sample may have arrived while
  // disabled (startup settings-restore race) or the request publisher may have
  // been torn down by a disable since the last reconcile. Direct call on the
  // GUI thread; enabled_ is already true so handleCatalog() proceeds. Warm-load
  // seeds tiles at reconciler version 0, so each enable re-requests the full
  // catalog set — a deliberate full-heal burst per enable (ADR-0006 D3), not
  // an incremental delta.
  if(last_catalog_)
    handleCatalog(*last_catalog_);
  writeSettings();
  updateDisplay();
  cached_image_ = QImage();
  update(boundingRect());
}

void SonarLiveCacheLayer::disableLiveCoverage()
{
  if(!enabled_)
    return;
  enabled_ = false;
  unsubscribeTiles();
  // [camp#172] Join any in-flight reload and drop the evicted-index bookkeeping — a
  // disabled layer keeps its in-memory + on-disk cache but stops the reload machinery.
  reload_watcher_.waitForFinished();
  evicted_fine_indices_.clear();
  reload_attempted_.clear();
  last_reload_viewport_ = QRectF();
  writeSettings();
  updateDisplay();
  update(boundingRect());
}

void SonarLiveCacheLayer::warmLoad()
{
  if(!level_)
  {
    // Without a known level we can't recover a GridIndex from a cached GeoTIFF.
    // Probe one cached file's level from its filename `<level>_<row>_<col>.tif`.
    QDir dir(QString::fromStdString(cache_dir_));
    const QStringList files = dir.entryList(QStringList() << "*.tif", QDir::Files);
    for(const QString& name : files)
    {
      const QString base = name.section('.', 0, 0);
      bool ok = false;
      const int lvl = base.section('_', 0, 0).toInt(&ok);
      if(ok && lvl >= 0 && lvl < 256)
      {
        level_ = static_cast<std::uint8_t>(lvl);
        break;
      }
    }
  }
  if(!level_)
    return;   // nothing cached yet

  // [camp#160] Load fine tiles INCREMENTALLY (not SonarLiveTile::loadCacheDir, which
  // would materialize the whole disk cache in one vector) and trim to the budget as
  // we go, so a large cache can't spike memory during warm-load — the salmon
  // accelerant (#153). An index already resident is kept (memory is >= the disk
  // copy), so a re-enable never displaces a live Entry — which also means no GL
  // context is needed here (no displaced texture to free; evictIfOverBudget() manages
  // its own context).
  const gggs::Level level(*level_);
  QDir fine_dir(QString::fromStdString(cache_dir_));
  int since_trim = 0;
  for(const QString& name : fine_dir.entryList(QStringList() << "*.tif", QDir::Files))
  {
    auto tile = SonarLiveTile::loadFromGeoTiff(fine_dir.filePath(name).toStdString(), level);
    if(!tile || !tile->index().valid())
      continue;
    const gggs::GridIndex index = tile->index();
    if(tiles_.count(index))
      continue;   // keep the resident (>= disk) copy; skip re-load on re-enable
    // Seed the reconciler at version 0: "have something" — older than any real
    // catalog version, so the next catalog re-requests it if the boat is newer, and
    // the prune gate never deletes it spuriously (ADR-0006 D3).
    reconciler_.markHave(index, 0);
    tiles_.insert_or_assign(index, Entry{std::move(*tile), nullptr, true, 0});
    if(++since_trim >= 64)   // cap the load peak at ~budget + 64 tiles
    {
      since_trim = 0;
      evictIfOverBudget();
    }
  }

  // [camp#160] Warm-load the overview pyramid. Overview tiles span multiple coarse
  // levels, so recover each file's level from its `<level>_<row>_<col>.tif` stem
  // rather than assuming the fine level. Kept resident (never evicted).
  QDir overview_dir(
    QDir(QString::fromStdString(cache_dir_)).filePath(QString::fromStdString(kOverviewSubdir)));
  for(const QString& name : overview_dir.entryList(QStringList() << "*.tif", QDir::Files))
  {
    bool ok = false;
    const int lvl = name.section('.', 0, 0).section('_', 0, 0).toInt(&ok);
    if(!ok || lvl < 0 || lvl >= static_cast<int>(gggs::levels.size()))
      continue;
    auto tile = SonarLiveTile::loadFromGeoTiff(overview_dir.filePath(name).toStdString(),
                                               gggs::Level(static_cast<std::uint8_t>(lvl)));
    if(tile && tile->index().valid())
      overview_tiles_.insert_or_assign(tile->index(),
                                       Entry{std::move(*tile), nullptr, true, 0});
  }

  if(band_name_.empty())
    band_name_ = defaultBand();
  recomputeBounds();
  resetAutoRange();
  foldAutoRange();
  updateDisplay();

  // [camp#160] Trim the freshly warm-loaded fine tiles to the budget before the
  // tile stream is subscribed, so a large disk cache can't spike memory on enable.
  evictIfOverBudget();
}

// ------------------------------ message handlers -----------------------------

void SonarLiveCacheLayer::handleTile(const marine_interfaces::msg::SonarVisualizationTile& msg)
{
  if(!enabled_)
    return;

  // [camp#170] Validate dimensions BEFORE any allocation or state mutation: a
  // single oversized/corrupt message would otherwise allocate
  // width*height*bands floats on the GUI thread on receipt (the 2026-07-23
  // operator-station crash) — the ADR-0010 eviction budget only accounts for
  // tiles after they are resident. Per-edge cap plus a combined byte ceiling:
  // per-edge alone still admits 4096x4096 x 64 bands x 4B ~= 4 GiB. 256 MiB
  // admits any legitimate <=4-band full-size tile (64 MiB/band at 4096^2).
  // kMaxImageEdge doubles as the ingest ceiling (it is the render clamp): a
  // producer emitting larger tiles would loop reject -> catalog re-request, so
  // raising producer tile size means raising the clamp too. The byte ceiling
  // counts msg.bands.size() even though applyPatch allocates per unique band
  // name — duplicate-named bands over-count, which errs on rejection (safe).
  constexpr std::size_t kMaxBandCount = 64;
  constexpr std::size_t kMaxTileBytes = std::size_t(256) * 1024 * 1024;
  const std::size_t tile_bytes = std::size_t(msg.width) * msg.height *
                                 msg.bands.size() * sizeof(float);
  if(msg.width == 0 || msg.height == 0 ||
     msg.width > kMaxImageEdge || msg.height > kMaxImageEdge ||
     msg.bands.size() > kMaxBandCount || tile_bytes > kMaxTileBytes)
  {
    qWarning().noquote() << "[live coverage" << QString::fromStdString(base_namespace_)
                         << "] rejected tile with absurd dimensions"
                         << msg.width << "x" << msg.height
                         << "bands:" << msg.bands.size();
    return;
  }

  const gggs::GridIndex index = gridIndexFromTileIndex(msg.index);
  if(!index.valid())
    return;
  level_ = msg.index.level;

  const marine_tiled_raster_store::TileVersion version = toNanoseconds(msg.header);
  // Newest-wins: a strictly-older patch is stale (the catalog/TileRequest path
  // heals any genuinely lost sub-window by re-sending the tile in full).
  const auto held = reconciler_.versionOf(index);
  if(held && version < *held)
    return;

  auto it = tiles_.find(index);
  const bool is_new = (it == tiles_.end());
  if(is_new)
    it = tiles_.emplace(index,
                        Entry{SonarLiveTile(index, msg.width, msg.height), nullptr, true, 0})
           .first;
  // [camp#172] The index is resident again (live re-send) — drop it from the
  // on-demand reload set so a pending reload doesn't redundantly reload it.
  evicted_fine_indices_.erase(index);
  Entry& entry = it->second;
  entry.tile.applyPatch(msg);
  entry.texture_dirty = true;
  entry.last_access_seq = ++access_seq_;   // [camp#160] freshest — LRU fallback
  reconciler_.markHave(index, entry.tile.version());

  scheduleWriteThrough(entry.tile);

  // [camp#160] Bound the resident fine-tile cache: over budget, the farthest
  // tiles from the viewport fold into their overview parent, persist, and drop.
  // May erase entries other than this one (or this one if it is the farthest and
  // we are over budget), so do not touch `entry` after this call.
  evictIfOverBudget();

  if(band_name_.empty())
    band_name_ = defaultBand();
  if(is_new)
    recomputeBounds();
  foldAutoRange();
  updateDisplay();
  cached_image_ = QImage();
  update(boundingRect());
}

void SonarLiveCacheLayer::handleCatalog(const marine_interfaces::msg::TileCatalog& msg)
{
  // Learn the level even while inactive (used for availability + later warm-load).
  if(!level_ && !msg.entries.empty())
    level_ = msg.entries.front().index.level;

  // [camp#169] Always buffer, even while disabled: the transient-local depth-1
  // subscription delivers its latched sample exactly once — discarding it here
  // while disabled means no reconcile ever fires (the boat's catalog is stable,
  // so nothing re-delivers it). enableLiveCoverage() replays this buffer.
  last_catalog_ = msg;

  if(!enabled_)
  {
    updateDisplay();   // refresh availability count
    return;
  }

  // Anti-entropy: converge the cache to exactly the catalog set (ADR-0006 D4).
  const marine_tiled_raster_store::TileCatalog catalog = toReconcilerCatalog(msg);
  const marine_tiled_raster_store::ReconcileResult result = reconciler_.reconcile(catalog);

  publishRequest(result.to_request);

  // [camp#134] A pruned Entry owns a QOpenGLTexture whose dtor frees GPU resources
  // ONLY under a current GL context — erasing it here without one leaks the texture
  // as tiles churn (the boat keeps producing; the catalog keeps pruning). Make the
  // renderer's context current for the erase loop, exactly like the dtor's teardown.
  // Gate on hasContext() (mirrors GggsTileLayer::applyBand): with no context yet no
  // texture was ever uploaded, so a plain erase is harmless and we skip makeCurrent()
  // rather than forcing the offscreen context into existence. If the context EXISTS
  // but makeCurrent() fails, the textures stay alive but a plain erase still mirrors
  // the dtor's null-context guard.
  bool pruned = false;
  const bool gl_current =
    !result.to_prune.empty() && renderer_.hasContext() && renderer_.makeCurrent();
  for(const auto& index : result.to_prune)
  {
    auto it = tiles_.find(index);
    if(it != tiles_.end())
    {
      it->second.texture.reset();   // free the GPU texture under the current context
      tiles_.erase(it);
      pruned = true;
    }
    // Delete the disk copy too (best-effort).
    namespace fs = std::filesystem;
    const std::string stem = std::to_string(static_cast<int>(index.level())) + "_" +
                             std::to_string(index.row()) + "_" +
                             std::to_string(index.column()) + ".tif";
    std::error_code ec;
    fs::remove(fs::path(cache_dir_) / stem, ec);
    // [camp#172] The disk copy is gone, so this index is no longer reloadable — drop
    // it from the on-demand reload set, or kickReload() would snapshot it and
    // loadFromGeoTiff() would fail forever with the index never cleared.
    evicted_fine_indices_.erase(index);
    reconciler_.drop(index);
  }
  if(gl_current)
    renderer_.doneCurrent();
  if(pruned)
  {
    // Reset before re-folding so a pruned tile's extreme min/max can't linger in
    // data_min_/data_max_ — the range reflects only the surviving tiles (mirrors
    // the band-switch reset in setBandName()).
    recomputeBounds();
    resetAutoRange();
    foldAutoRange();
    cached_image_ = QImage();
    update(boundingRect());
  }
  updateDisplay();
}

void SonarLiveCacheLayer::scheduleWriteThrough(const SonarLiveTile& tile,
                                               const std::string& subdir)
{
  // GUI thread. Coalesce per tile: snapshot the latest state and launch a worker
  // only if this tile is idle. If a worker is already in flight for this tile, mark
  // it dirty — onWriteThroughFinished() re-launches once with the latest snapshot,
  // so two workers never race on the shared <stem>.tif.tmp path (the prior bug),
  // and intermediate patches are coalesced into a single follow-up write.
  WriteState& ws = write_states_[tile.index()];
  ws.pending = tile;   // latest wins
  ws.dirty = true;
  ws.subdir = subdir;  // [camp#160] "" (fine, root) or "overviews" (pyramid)
  if(!ws.in_flight)
    startWriteThrough(tile.index());
}

void SonarLiveCacheLayer::startWriteThrough(const gggs::GridIndex& index)
{
  // GUI thread. Move the latest snapshot into a self-contained worker (no `this`
  // deref) and track its watcher so the dtor can join it. `pending` is repopulated
  // by the next scheduleWriteThrough() before it is read again, so moving is safe.
  WriteState& ws = write_states_[index];
  ws.in_flight = true;
  ws.dirty = false;
  auto* watcher = new QFutureWatcher<void>(this);
  write_watchers_.push_back(watcher);
  connect(watcher, &QFutureWatcher<void>::finished, this,
          [this, index, watcher]() { onWriteThroughFinished(index, watcher); });
  // [camp#160] Destination = cache_dir_[/subdir]; the worker creates it.
  const std::string dir = ws.subdir.empty()
    ? cache_dir_
    : (std::filesystem::path(cache_dir_) / ws.subdir).string();
  watcher->setFuture(QtConcurrent::run(writeTileToCache, std::move(*ws.pending), dir));
}

void SonarLiveCacheLayer::onWriteThroughFinished(const gggs::GridIndex& index,
                                                 QFutureWatcher<void>* watcher)
{
  // GUI thread. Untrack + delete the finished worker's watcher.
  write_watchers_.erase(
    std::remove(write_watchers_.begin(), write_watchers_.end(), watcher),
    write_watchers_.end());
  watcher->deleteLater();
  if(shutting_down_)
    return;
  auto it = write_states_.find(index);
  if(it == write_states_.end())
    return;
  it->second.in_flight = false;
  if(it->second.dirty)
    startWriteThrough(index);   // coalesced: write the newest patch that arrived
  else
    write_states_.erase(it);    // clean: drop the per-tile state
}

// -------------------------- eviction / overview pyramid ----------------------

std::size_t SonarLiveCacheLayer::entryBytes(const Entry& e)
{
  // Resident footprint of one entry: CPU band data (always present) plus the selected
  // band's GL texture when uploaded.
  std::size_t bytes = 0;
  for(const auto& name : e.tile.bandNames())
    if(const SonarLiveBand* band = e.tile.band(name))
      bytes += band->data.size() * sizeof(float);
  if(e.texture)
    bytes += static_cast<std::size_t>(e.tile.width()) * e.tile.height() * sizeof(float);
  return bytes;
}

std::size_t SonarLiveCacheLayer::fineResidentBytes() const
{
  std::size_t bytes = 0;
  for(const auto& item : tiles_)
    bytes += entryBytes(item.second);
  return bytes;
}

std::size_t SonarLiveCacheLayer::overviewResidentBytes() const
{
  std::size_t bytes = 0;
  for(const auto& item : overview_tiles_)
    bytes += entryBytes(item.second);
  return bytes;
}

std::size_t SonarLiveCacheLayer::accountedBytes() const
{
  // Total resident footprint over BOTH fine tiles and overview (pyramid) tiles — the
  // overviews grow with survey area, so they must count toward the budget or they'd be
  // an unbounded second cache (ADR-0010 D1).
  return fineResidentBytes() + overviewResidentBytes();
}

std::optional<QPointF> SonarLiveCacheLayer::currentViewCentre() const
{
  // Viewport centre in Web-Mercator scene coords (the eviction reference point).
  // Null when headless (no scene/view attached, e.g. the render tests).
  const QGraphicsScene* graphics_scene = scene();
  if(!graphics_scene)
    return std::nullopt;
  const QList<QGraphicsView*> views = graphics_scene->views();
  if(views.isEmpty() || !views.first())
    return std::nullopt;
  QGraphicsView* view = views.first();
  return view->mapToScene(view->viewport()->rect()).boundingRect().center();
}

void SonarLiveCacheLayer::foldIntoParent(const SonarLiveTile& fine)
{
  // Fold a fine tile into its coarse parent and recurse up to level 0, so a zoomed-out
  // view always has coverage even after the fine tiles are evicted. Each overview tile
  // is built at the fine tile's own width/height — the fixed uniform
  // TiledRasterTile::edge — so this is the standard half-resolution-per-level pyramid
  // and CONVERGES with the uma shared fold engine (overview_builder.hpp::buildParentTile
  // folds every parent at the fixed TiledRasterTile<T>::edge with the MEAN cell policy):
  // identical fidelity to the merged store's pyramid at the same zoom (ADR-0010 D3,
  // camp#171). foldChild() requires parent and child to be the same size (it maps each
  // child into a 1/4 sub-window), so the parent MUST match fine.width()/height(). They
  // persist under the `overviews/` cache sub-dir (the uma sidecar layout) and are never
  // added to the reconciler (a local derived product; prune-on-absence must not touch
  // them).
  const gggs::GridIndex parent_index = gggs::parent(fine.index());
  if(!parent_index.valid())
    return;
  auto it = overview_tiles_.find(parent_index);
  if(it == overview_tiles_.end())
    it = overview_tiles_
           .emplace(parent_index,
                    Entry{SonarLiveTile(parent_index, fine.width(), fine.height()),
                          nullptr, true, 0})
           .first;
  Entry& overview = it->second;
  overview.tile.foldChild(fine);
  overview.texture_dirty = true;
  scheduleWriteThrough(overview.tile, kOverviewSubdir);
  foldIntoParent(overview.tile);   // build the full chain to level 0
}

void SonarLiveCacheLayer::evictIfOverBudget()
{
  if(vram_budget_bytes_ == 0 || accountedBytes() <= vram_budget_bytes_)
    return;

  if(!eviction_warned_)
  {
    qWarning().noquote() << "[live coverage" << QString::fromStdString(base_namespace_)
                         << "] resident tile budget exceeded ("
                         << qulonglong(accountedBytes()) << ">"
                         << qulonglong(vram_budget_bytes_)
                         << "bytes) — shedding to the overview pyramid";
    eviction_warned_ = true;
  }

  // Order candidate indices farthest-from-viewport-centre first (view-based LOD);
  // LRU by last_access_seq (oldest first) when headless (no view attached).
  const std::optional<QPointF> centre = currentViewCentre();
  auto orderByDistance = [&](const std::map<gggs::GridIndex, Entry>& pool,
                             std::vector<gggs::GridIndex> keys)
  {
    std::sort(keys.begin(), keys.end(),
              [&](const gggs::GridIndex& a, const gggs::GridIndex& b)
              {
                const Entry& ea = pool.at(a);
                const Entry& eb = pool.at(b);
                if(centre)
                  return tileSceneDistanceSquared(ea.tile, *centre) >
                         tileSceneDistanceSquared(eb.tile, *centre);
                return ea.last_access_seq < eb.last_access_seq;
              });
    return keys;
  };

  // Fine tiles and overviews are already persisted (handleTile / foldIntoParent write
  // through), so eviction just frees memory — the disk cache is the durable backing
  // (ADR-0010 D2). Reconciler markHave is kept (possession on disk), NOT dropped, so
  // eviction can't trigger a re-request/re-evict churn (#71): tiles_ = residency, the
  // reconciler = possession. Free GL textures under the renderer's context.
  const bool gl_current = renderer_.hasContext() && renderer_.makeCurrent();
  bool evicted = false;

  // Phase 1 — evict fine tiles (the detail) farthest-first, folding each into its
  // coarse parent so its coverage survives at lower resolution.
  {
    std::vector<gggs::GridIndex> keys;
    keys.reserve(tiles_.size());
    for(const auto& item : tiles_)
      keys.push_back(item.first);
    for(const gggs::GridIndex& index : orderByDistance(tiles_, std::move(keys)))
    {
      if(accountedBytes() <= vram_budget_bytes_)
        break;
      auto it = tiles_.find(index);
      if(it == tiles_.end())
        continue;
      foldIntoParent(it->second.tile);   // degrade to coarse parent (persists it)
      it->second.texture.reset();
      tiles_.erase(it);
      // [camp#172] Record it for on-demand reload: its disk copy survives (persisted by
      // handleTile's write-through), so panning back can reload it (ADR-0010 D2).
      evicted_fine_indices_.insert(index);
      evicted = true;
    }
  }

  // Phase 2 — if the pyramid itself is still over budget, evict the numerous
  // near-fine overview tiles farthest-first, but NEVER the coarse apex
  // (level <= kApexProtectLevel) so a whole-survey zoom-out always has coverage. An
  // evicted overview's data already lives in its coarser ancestors (built by the same
  // fold chain), so dropping it loses no coverage — just some mid-zoom fidelity.
  if(accountedBytes() > vram_budget_bytes_)
  {
    std::vector<gggs::GridIndex> keys;
    for(const auto& item : overview_tiles_)
      if(item.first.level() > kApexProtectLevel)
        keys.push_back(item.first);
    for(const gggs::GridIndex& index : orderByDistance(overview_tiles_, std::move(keys)))
    {
      if(accountedBytes() <= vram_budget_bytes_)
        break;
      auto it = overview_tiles_.find(index);
      if(it == overview_tiles_.end())
        continue;
      it->second.texture.reset();
      overview_tiles_.erase(it);
      evicted = true;
    }
  }
  // May intentionally return still-over-budget when only the protected apex remains
  // (apex bytes > budget) — that is the O(small) floor that guarantees zoom-out
  // coverage, not a leak (ADR-0010 D1 / Consequences).

  if(gl_current)
    renderer_.doneCurrent();

  if(evicted)
  {
    recomputeBounds();
    resetAutoRange();
    foldAutoRange();
    cached_image_ = QImage();
    update(boundingRect());
  }
}

// --------------------------- on-demand reload (#172) -------------------------

bool SonarLiveCacheLayer::hasUnloadedVisibleTiles(const QRectF& viewport_scene) const
{
  // [camp#172 / ADR-0013] Does the viewport expose an evicted fine tile whose disk copy
  // is not resident? GUI thread. The tile is gone, so test the GGGS extent of the index
  // directly (same extent math as itemsIntersecting's per-tile clip test). A null
  // viewport (headless with no clip) matches nothing — reload is viewport-driven.
  if(viewport_scene.isNull())
    return false;
  for(const gggs::GridIndex& index : evicted_fine_indices_)
    if(indexSceneRect(index).intersects(viewport_scene))
      return true;
  return false;
}

void SonarLiveCacheLayer::kickReload(const QRectF& viewport_scene)
{
  // [camp#172] Snapshot the visible evicted indices + cache location into a
  // self-contained worker and launch it (mirrors GggsTileLayer::loadTiles()). Record
  // the kick's viewport so paint()'s re-kick fires only when the viewport actually
  // moved, and the attempted set so onReloadFinished() clears it whether or not each
  // load succeeds. Requires a known fine level to recover a GridIndex from a GeoTIFF.
  if(reload_watcher_.isRunning() || !level_ || viewport_scene.isNull())
    return;
  std::vector<gggs::GridIndex> visible;
  for(const gggs::GridIndex& index : evicted_fine_indices_)
    if(indexSceneRect(index).intersects(viewport_scene))
      visible.push_back(index);
  if(visible.empty())
    return;
  last_reload_viewport_ = viewport_scene;
  reload_attempted_ = visible;
  reload_watcher_.setFuture(
    QtConcurrent::run(reloadTilesFromCache, std::move(visible), cache_dir_, *level_));
}

void SonarLiveCacheLayer::onReloadFinished()
{
  // GUI thread (Qt-queued from the watcher's finished signal). Insert the reloaded fine
  // tiles back into tiles_, then maintain the budget — ADR-0010 D6 hysteresis (paint()
  // only kicks under 0.75x budget) keeps evictIfOverBudget() from immediately shedding
  // what we just reloaded.
  if(shutting_down_)
    return;
  // Idempotent + safe against a spurious call with no in-flight reload: reload_attempted_
  // is non-empty exactly between a kickReload() and the matching onReloadFinished()
  // (this clears it below). Guards both the double-invoke (explicit waitForReload() call
  // plus the queued finished signal) and reading result() on a never-set future.
  if(reload_attempted_.empty())
    return;
  const std::vector<SonarLiveTile> loaded = reload_watcher_.future().result();

  // Clear EVERY attempted index (loaded or not): a permanently-unloadable index must
  // not keep hasUnloadedVisibleTiles() true and re-kick on the next viewport move.
  for(const gggs::GridIndex& index : reload_attempted_)
    evicted_fine_indices_.erase(index);
  reload_attempted_.clear();

  bool inserted = false;
  for(const SonarLiveTile& tile : loaded)
  {
    const gggs::GridIndex index = tile.index();
    // A live re-send may have re-added it while the worker ran — newest resident wins.
    if(tiles_.count(index))
      continue;
    tiles_.insert_or_assign(index, Entry{tile, nullptr, true, ++access_seq_});
    inserted = true;
  }

  if(!inserted)
  {
    updateDisplay();
    return;
  }

  if(band_name_.empty())
    band_name_ = defaultBand();
  recomputeBounds();
  resetAutoRange();
  foldAutoRange();
  evictIfOverBudget();   // maintain the budget; D6 hysteresis prevents a re-evict storm
  updateDisplay();
  cached_image_ = QImage();
  update(boundingRect());
}

void SonarLiveCacheLayer::waitForReload(const QRectF& viewport_scene)
{
  // [camp#172] Test/headless analogue of paint()'s reload kick + GggsTileLayer's
  // waitForLoad(): apply the SAME gate as paint() (idle + budget headroom via the D6
  // hysteresis + viewport-moved-since-last-kick + an evicted visible tile), kick, then
  // join and run the ready-slot so the reload is visible on return. Mirroring the full
  // gate lets a test exercise the hysteresis guard by controlling the budget.
  if(!reload_watcher_.isRunning() &&
     vram_budget_bytes_ > 0 &&
     accountedBytes() < static_cast<std::size_t>(vram_budget_bytes_ * kReloadHysteresisFactor) &&
     viewport_scene != last_reload_viewport_ &&
     hasUnloadedVisibleTiles(viewport_scene))
    kickReload(viewport_scene);
  reload_watcher_.waitForFinished();
  onReloadFinished();
}

// ------------------------------ extent / range -------------------------------

void SonarLiveCacheLayer::recomputeBounds()
{
  prepareGeometryChange();
  QRectF bounds;
  bool first = true;
  auto expand = [&](const SonarLiveTile& tile)
  {
    const QPointF lo = web_mercator::geoToMap(QGeoCoordinate(tile.minLat(), tile.minLon()));
    const QPointF hi = web_mercator::geoToMap(QGeoCoordinate(tile.maxLat(), tile.maxLon()));
    const QRectF rect = QRectF(lo, hi).normalized();
    bounds = first ? rect : bounds.united(rect);
    first = false;
  };
  for(const auto& entry : tiles_)
    expand(entry.second.tile);
  // [camp#160] Include overviews so the extent stays correct after fine tiles are
  // evicted (an all-evicted region is still covered by its resident overview).
  for(const auto& entry : overview_tiles_)
    expand(entry.second.tile);
  scene_bounds_ = bounds;
  if(!scene_bounds_.isNull())
  {
    // North-up anchored at the NW corner with a negative-Y transform (the camp_map
    // raster convention; matches GggsTileLayer).
    setTransform(QTransform::fromScale(1.0, -1.0));
    setPos(QPointF(scene_bounds_.left(), scene_bounds_.bottom()));
  }
}

void SonarLiveCacheLayer::resetAutoRange()
{
  data_min_ = 1.0;
  data_max_ = 0.0;
}

void SonarLiveCacheLayer::foldAutoRange()
{
  bool first = (data_min_ > data_max_);
  auto fold = [&](const Entry& e)
  {
    const SonarLiveBand* band = e.tile.band(band_name_);
    if(!band || band->data_min > band->data_max)
      return;
    if(first || band->data_min < data_min_) data_min_ = band->data_min;
    if(first || band->data_max > data_max_) data_max_ = band->data_max;
    first = false;
  };
  for(const auto& entry : tiles_)
    fold(entry.second);
  // [camp#160] Fold overview ranges too so the colormap tracks the full visible
  // extent when only overviews remain for a region.
  for(const auto& entry : overview_tiles_)
    fold(entry.second);
  // [camp#142] Keep the Auto resolved range current with the freshly-folded extents
  // (a no-op while the operator holds a Manual override). camp#138 coordination:
  // this update_auto() call site is where #138's fold changes meet this PR — the
  // override is applied at render time and is transparent to the accumulation.
  if(data_min_ <= data_max_)
    range_model_.update_auto(float(data_min_), float(data_max_));
}

std::string SonarLiveCacheLayer::defaultBand() const
{
  // Prefer "depth" when present (the primary bathy band); else the first band.
  for(const auto& entry : tiles_)
    if(entry.second.tile.band("depth"))
      return "depth";
  for(const auto& entry : tiles_)
  {
    const auto names = entry.second.tile.bandNames();
    if(!names.empty())
      return names.front();
  }
  return std::string();
}

void SonarLiveCacheLayer::updateDisplay()
{
  if(enabled_)
    setStatus(QString("(live: %1 fine + %2 overview tiles)")
                .arg(qulonglong(tiles_.size()))
                .arg(qulonglong(overview_tiles_.size())));
  else
    setStatus("(available)");
}

// --------------------------------- rendering ---------------------------------

QRectF SonarLiveCacheLayer::boundingRect() const
{
  return QRectF(QPointF(0.0, 0.0), scene_bounds_.size());
}

QOpenGLTexture* SonarLiveCacheLayer::textureFor(Entry& entry)
{
  // [camp#160] Rendering a tile marks it recently used (the LRU eviction fallback
  // approximates "in view"). Harmless for overview entries (never evicted).
  entry.last_access_seq = ++access_seq_;
  const SonarLiveBand* band = entry.tile.band(band_name_);
  if(!band || band->data.empty())
    return nullptr;
  if(entry.texture && !entry.texture_dirty)
    return entry.texture.get();
  // (Re)upload the selected band as an R32F value texture. The Nearest filter (so
  // the exact-equality NoData discard never blends across a sentinel boundary) is
  // now applied by the renderer at draw time (camp#134); we just upload the data
  // and the clamp wrap.
  entry.texture = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
  entry.texture->setFormat(QOpenGLTexture::R32F);
  entry.texture->setSize(entry.tile.width(), entry.tile.height());
  entry.texture->setMipLevels(1);
  entry.texture->allocateStorage(QOpenGLTexture::Red, QOpenGLTexture::Float32);
  entry.texture->setData(QOpenGLTexture::Red, QOpenGLTexture::Float32, band->data.data());
  entry.texture->setWrapMode(QOpenGLTexture::ClampToEdge);
  entry.texture_dirty = false;
  return entry.texture.get();
}

QStringList SonarLiveCacheLayer::bands() const
{
  // [camp#134] Named live bands (union across held tiles), for the band picker.
  QStringList result;
  for(const auto& item : tiles_)
    for(const auto& name : item.second.tile.bandNames())
    {
      const QString q = QString::fromStdString(name);
      if(!result.contains(q))
        result << q;
    }
  return result;
}

raster::RasterBandMeta SonarLiveCacheLayer::metadata(const QString& band) const
{
  // [camp#134] Surface the selected band's dequantized NoData sentinel from the
  // first tile that carries it (the renderer reads NoData per item; this is for a
  // future unified band picker).
  raster::RasterBandMeta meta;
  const std::string name = band.toStdString();
  for(const auto& item : tiles_)
    if(const SonarLiveBand* b = item.second.tile.band(name))
    {
      meta.has_nodata = b->has_nodata;
      meta.nodata = b->nodata;
      break;
    }
  return meta;
}

QPair<float, float> SonarLiveCacheLayer::dataRange() const
{
  return {float(data_min_), float(data_max_)};
}

QList<raster::RasterFieldItem> SonarLiveCacheLayer::items()
{
  return itemsIntersecting(QRectF());
}

QList<raster::RasterFieldItem> SonarLiveCacheLayer::itemsIntersecting(
  const QRectF& clip_scene)
{
  // [camp#134] Collect the held tiles' selected band as Scalar items for the shared
  // renderer. Called with the renderer's GL context current (renderImage()), so
  // textureFor() may lazily (re)upload here. The geo->Web-Mercator warp lives in
  // the renderer; this only forwards each tile's lat/lon extent + NoData sentinel.
  // [camp#103] A non-null @p clip_scene keeps only tiles whose Web-Mercator extent
  // intersects it — tested BEFORE textureFor(), so offscreen tiles are neither
  // uploaded nor drawn. Both pools are filtered with the same predicate, keeping
  // the overviews-first order below intact.
  QList<raster::RasterFieldItem> result;
  result.reserve(int(overview_tiles_.size() + tiles_.size()));
  auto append = [&](Entry& entry)
  {
    if(!clip_scene.isNull())
    {
      const QPointF lo = web_mercator::geoToMap(
        QGeoCoordinate(entry.tile.minLat(), entry.tile.minLon()));
      const QPointF hi = web_mercator::geoToMap(
        QGeoCoordinate(entry.tile.maxLat(), entry.tile.maxLon()));
      if(!QRectF(lo, hi).normalized().intersects(clip_scene))
        return;
    }
    const SonarLiveBand* band = entry.tile.band(band_name_);
    if(!band)
      return;
    QOpenGLTexture* texture = textureFor(entry);
    if(!texture)
      return;
    raster::RasterFieldItem fi;
    fi.texture = texture;
    fi.format = raster::RasterFieldItem::Format::Scalar;
    fi.geographic = true;
    fi.west = entry.tile.minLon();
    fi.east = entry.tile.maxLon();
    fi.south = entry.tile.minLat();
    fi.north = entry.tile.maxLat();
    fi.has_nodata = band->has_nodata;
    fi.nodata = band->has_nodata ? band->nodata : 0.0f;
    result.push_back(fi);
  };
  // [camp#160] LOD fallback by draw order: overviews first (coarse->fine, the
  // std::map orders by GGGS level), then the fine tiles on top. Where a fine tile
  // is present it fully covers its parent; where it was evicted, the coarse parent
  // shows through instead of a blank gap.
  for(auto& item : overview_tiles_)
    append(item.second);
  for(auto& item : tiles_)
    append(item.second);
  return result;
}

QImage SonarLiveCacheLayer::renderImage(const QSize& size)
{
  return renderImage(size, scene_bounds_);
}

QImage SonarLiveCacheLayer::renderImage(const QSize& size, const QRectF& clip_bounds)
{
  if((tiles_.empty() && overview_tiles_.empty()) || data_min_ > data_max_ ||
     size.isEmpty() || clip_bounds.isEmpty())
    return QImage();
  // [camp#134] Make the renderer's context current, collect the held tiles
  // (uploading textures under it), then delegate the warp + draw.
  if(!renderer_.makeCurrent())
    return QImage();
  // [camp#103] Only tiles intersecting the clip contribute (both pools,
  // overviews-first order preserved — the ADR-0010 LOD fallback).
  const QList<raster::RasterFieldItem> draw = itemsIntersecting(clip_bounds);
  // [camp#142] Feed the resolved range (Auto tracks data_min_/data_max_; Manual is
  // the operator override) into the shader's u_min/u_max instead of the raw extents.
  const QImage image = renderer_.renderToImage(draw, clip_bounds, range_model_.lo(),
                                               range_model_.hi(), size);
  renderer_.doneCurrent();
  return image;
}

void SonarLiveCacheLayer::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*)
{
  if((tiles_.empty() && overview_tiles_.empty()) || data_min_ > data_max_)
    return;

  // [camp#103 / ADR-0011] Render only the viewport-visible clip of the extent,
  // sized to its on-screen pixels. Re-render when the size (zoom) OR the clip
  // (pan) changes; a viewport-sized FBO makes the per-frame pan re-render cheap.
  const raster::ViewportClip clip = raster::deriveViewportClip(
    painter, this, boundingRect(), scene_bounds_, kMaxImageEdge);

  // [camp#172 / ADR-0010 D2/D6] On-demand reload: if the viewport now exposes an evicted
  // fine tile and there is budget headroom (hysteresis, so the reload can't immediately
  // re-trigger eviction), kick a filtered reload. The moved-since-last-kick guard
  // (clip.scene != last_reload_viewport_) keeps a permanently-unloadable index from
  // re-kicking every frame — mirrors GggsTileLayer's demand-driven loader.
  if(!reload_watcher_.isRunning() &&
     vram_budget_bytes_ > 0 &&
     accountedBytes() < static_cast<std::size_t>(vram_budget_bytes_ * kReloadHysteresisFactor) &&
     clip.scene != last_reload_viewport_ &&
     hasUnloadedVisibleTiles(clip.scene))
  {
    kickReload(clip.scene);
  }

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

// ------------------------------- band / colormap -----------------------------

void SonarLiveCacheLayer::setColormap(const std::string& name)
{
  if(name == renderer_.colormap())
    return;
  renderer_.setColormap(name);   // re-bakes the LUT on next render
  cached_image_ = QImage();
  writeSettings();
  update(boundingRect());
}

void SonarLiveCacheLayer::setBandName(const std::string& name)
{
  if(name == band_name_ || name.empty())
    return;
  band_name_ = name;
  // The value texture is per-band: mark every tile's texture for re-upload, reset
  // and re-fold the auto-range over the new band.
  for(auto& item : tiles_)
    item.second.texture_dirty = true;
  resetAutoRange();
  foldAutoRange();
  cached_image_ = QImage();
  writeSettings();
  update(boundingRect());
}

void SonarLiveCacheLayer::setRangeOverride(float lo, float hi)
{
  // [camp#142] Pin the resolved range to the operator's [lo, hi] (set_manual swaps
  // an inverted pair so lo() <= hi()). Re-render with the new range + persist.
  range_model_.set_manual(lo, hi);
  cached_image_ = QImage();
  writeSettings();
  update(boundingRect());
}

void SonarLiveCacheLayer::resetRangeToAuto()
{
  // [camp#142] Return to data-driven Auto, then re-track the current extents so
  // lo()/hi() reflect the data without waiting for the next fold.
  range_model_.reset();
  if(data_min_ <= data_max_)
    range_model_.update_auto(float(data_min_), float(data_max_));
  cached_image_ = QImage();
  writeSettings();
  update(boundingRect());
}

// --------------------------------- context menu ------------------------------

void SonarLiveCacheLayer::contextMenu(QMenu* menu)
{
  Layer::contextMenu(menu);

  // [camp#121 / ADR-0006 D5] The opt-in gate: enabling subscribes to the
  // best-effort tile stream; disabling stops it. Default discovered state is
  // inactive so a slow link (#71) pays nothing until the operator asks.
  if(enabled_)
  {
    QAction* disable = menu->addAction("Disable live coverage");
    connect(disable, &QAction::triggered, this,
            &SonarLiveCacheLayer::disableLiveCoverage);
  }
  else
  {
    QAction* enable = menu->addAction("Enable live coverage");
    connect(enable, &QAction::triggered, this,
            &SonarLiveCacheLayer::enableLiveCoverage);
  }

  // [camp#132] Per-layer blit-smoothing opt-in (default OFF = Nearest, the
  // faithful-QA baseline).
  QAction* smooth_action = menu->addAction("Smooth interpolation");
  smooth_action->setCheckable(true);
  smooth_action->setChecked(smooth_interpolation_);
  connect(smooth_action, &QAction::triggered, this,
          [this](bool on) { setSmoothInterpolation(on); });

  // [camp#141] Expose the FULL marine_colormap registry, not just the legacy ramps.
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(const std::string& name : marine_colormap::palette_names())
  {
    QAction* action = colormap_menu->addAction(QString::fromStdString(name));
    action->setCheckable(true);
    action->setChecked(name == renderer_.colormap());
    connect(action, &QAction::triggered, this, [this, name]() { setColormap(name); });
  }

  // [camp#142 PR2] Colormap range override. Live bands are scalar (same gating as
  // the Colormap submenu above), so the action is offered unconditionally. Opens
  // the interactive colorbar — drag the handles or edit the bounds to pin a Manual
  // override, reset to track the data extents — the successor to PR1's sequential
  // numeric prompts.
  QAction* range_action = menu->addAction("Colormap range…");
  connect(range_action, &QAction::triggered, this, [this]()
  {
    raster::ColormapRangeState state;
    const auto idx = marine_colormap::palette_index(renderer_.colormap());
    state.palette_index = idx ? static_cast<int>(*idx) : 0;
    state.data_min = static_cast<float>(data_min_);
    state.data_max = static_cast<float>(data_max_);
    state.mode = range_model_.mode();
    state.lo = range_model_.lo();
    state.hi = range_model_.hi();
    raster::showColormapRangeDialog(
      nullptr, "Colormap range", state,
      [this](float lo, float hi) { setRangeOverride(lo, hi); },
      [this]() { resetRangeToAuto(); });
  });

  // Band picker over the union of band names across held tiles. Only shown when
  // there is more than one band to choose from.
  std::vector<std::string> names;
  for(const auto& item : tiles_)
    for(const auto& name : item.second.tile.bandNames())
      if(std::find(names.begin(), names.end(), name) == names.end())
        names.push_back(name);
  if(names.size() > 1)
  {
    QMenu* band_menu = menu->addMenu("Band");
    for(const auto& name : names)
    {
      QAction* action = band_menu->addAction(QString::fromStdString(name));
      action->setCheckable(true);
      action->setChecked(name == band_name_);
      connect(action, &QAction::triggered, this, [this, name]() { setBandName(name); });
    }
  }
}

// --------------------------------- persistence -------------------------------

void SonarLiveCacheLayer::setSmoothInterpolation(bool smooth)
{
  if(smooth == smooth_interpolation_)
    return;
  smooth_interpolation_ = smooth;
  writeSettings();
  update(boundingRect());   // blit-hint-only change: no re-render needed
}

void SonarLiveCacheLayer::readSettings()
{
  Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(settingsKey());
  // [camp#132] Persisted blit-smoothing opt-in (default OFF = Nearest).
  smooth_interpolation_ = settings.value("smooth_interpolation", false).toBool();
  // Default discovered layers OFF in the tree (like GGGS tile-sets) — the operator
  // turns coverage on explicitly.
  setVisible(settings.value("visible", false).toBool());
  // [camp#141] Persisted palette name; case-insensitive read + registry-validated
  // (unknown -> grayscale), mirroring GggsTileLayer.
  std::string colormap = settings.value(
    "colormap", QString::fromStdString(renderer_.colormap())).toString().toLower().toStdString();
  if(!marine_colormap::palette_index(colormap))
    colormap = "grayscale";
  const std::string band = settings.value("band", QString::fromStdString(band_name_))
                             .toString().toStdString();
  const bool was_enabled = settings.value("live_enabled", false).toBool();
  // [camp#142] Persisted colormap range. "manual" restores the operator override;
  // anything else (default "auto") leaves the data-driven Auto range. Honor Manual
  // only when both extents are present too — a partial/corrupt entry falls back to
  // Auto rather than snapping to the [0,1] read-defaults.
  const QString range_mode = settings.value("range_mode", "auto").toString();
  const bool has_manual_range = range_mode == "manual" &&
    settings.contains("range_min") && settings.contains("range_max");
  const float range_min = settings.value("range_min", 0.0).toFloat();
  const float range_max = settings.value("range_max", 1.0).toFloat();
  settings.endGroup();
  settings.endGroup();

  if(colormap != renderer_.colormap())
  {
    renderer_.setColormap(colormap);
    cached_image_ = QImage();
  }
  if(!band.empty())
    band_name_ = band;
  // [camp#142] Apply the persisted range. A Manual override is independent of the
  // data extents and is restored as-is; "auto" leaves the model tracking the data.
  if(has_manual_range)
    range_model_.set_manual(range_min, range_max);
  else
    range_model_.reset();
  // [ADR-0006 D5] An enabled source re-subscribes on warm restart; a never-enabled
  // one stays passive. enableLiveCoverage() persists, which is a harmless re-write.
  if(was_enabled && !enabled_)
    enableLiveCoverage();
}

void SonarLiveCacheLayer::writeSettings()
{
  Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(settingsKey());
  settings.setValue("colormap", QString::fromStdString(renderer_.colormap()));
  settings.setValue("band", QString::fromStdString(band_name_));
  settings.setValue("live_enabled", enabled_);
  settings.setValue("smooth_interpolation", smooth_interpolation_);   // [camp#132]
  // [camp#142] Persist the colormap range mode + bounds so a Manual override (and
  // its [lo, hi]) survives a restart; Auto persists as "auto".
  settings.setValue("range_mode",
                    range_model_.mode() == marine_colormap::RangeMode::Manual ? "manual"
                                                                              : "auto");
  settings.setValue("range_min", range_model_.lo());
  settings.setValue("range_max", range_model_.hi());
  settings.endGroup();
  settings.endGroup();
}

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp
