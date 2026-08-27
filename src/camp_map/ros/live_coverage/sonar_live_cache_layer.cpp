#include "sonar_live_cache_layer.h"
#include "crash_handler.h"

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
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <set>
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

// [field 2026-08-27] The cache filename stem for a tile index, `<level>_<row>_<col>` —
// the single formula behind every cache path (fine tiles at the cache-dir root,
// overviews under `overviews/`). It was open-coded at three sites; the pyramid prune
// adds more, and a silent divergence between the writer's name and a deleter's name
// would orphan files rather than fail loudly.
std::string tileStem(const gggs::GridIndex& index)
{
  return std::to_string(static_cast<int>(index.level())) + "_" +
         std::to_string(index.row()) + "_" + std::to_string(index.column());
}

// [field 2026-08-27] Cache path of a fine tile (cache-dir root) and of an overview tile
// (the `overviews/` sidecar sub-dir).
std::filesystem::path fineTilePath(const std::string& cache_dir, const gggs::GridIndex& index)
{
  return std::filesystem::path(cache_dir) / (tileStem(index) + ".tif");
}

std::filesystem::path overviewTilePath(const std::string& cache_dir,
                                       const gggs::GridIndex& index)
{
  return std::filesystem::path(cache_dir) / kOverviewSubdir / (tileStem(index) + ".tif");
}

// [field 2026-08-27] Inverse of tileStem(): recover a GridIndex from a cache filename
// stem. Returns the invalid sentinel when the stem does not parse in full or does not
// name a real grid — the disk sweep then leaves that file alone rather than guessing,
// since writeTileToCache() is the only writer and only ever emits this shape.
gggs::GridIndex indexFromStem(const std::string& stem)
{
  int level = 0;
  unsigned long long row = 0;
  unsigned long long col = 0;
  int consumed = 0;
  if(std::sscanf(stem.c_str(), "%d_%llu_%llu%n", &level, &row, &col, &consumed) != 3 ||
     consumed != static_cast<int>(stem.size()))
    return gggs::GridIndex();
  if(level < 0 || level > 255 ||
     row > 0xffffffffull || col > 0xffffffffull)
    return gggs::GridIndex();
  marine_interfaces::msg::TileIndex wire;
  wire.level = static_cast<std::uint8_t>(level);
  wire.row = static_cast<std::uint32_t>(row);
  wire.col = static_cast<std::uint32_t>(col);
  return gridIndexFromTileIndex(wire);
}

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

// [field 2026-08-27 / ADR-0010 D5] The sub-rect of @p ancestor's texture that
// @p descendant occupies, in normalized texture coordinates (u west->east, v
// north->south — texture row 0 is north, as SonarLiveTile stores it and as the
// renderer's vertex generation assumes). This is what clips a coarse placeholder to
// one fine tile's footprint instead of stretching the whole overview over it.
//
// Derived from GGGS index arithmetic, not from an extent ratio: a level's grid rows
// and columns both double per level (the ±72/±80 `latitudeScaleFactor` bands are
// bounded by whole grid rows at every level, so an ancestor and its descendants are
// always inside one band and share the scale factor), which makes the window an
// exact power-of-two fraction — no floating-point extent division, and no drift when
// the walk spans many levels. Rows count NORTHWARD while v counts southward, hence
// the row flip.
//
// Returns false — caller draws nothing — when @p descendant is not really inside
// @p ancestor at a finer level. That is the polar-band case the fold itself already
// declines to handle (ADR-0010's `foldChild` boundary follow-up), and a silent wrong
// window would paint one tile's data over another's footprint.
bool textureWindowOf(const gggs::GridIndex& ancestor, const gggs::GridIndex& descendant,
                     float& u0, float& v0, float& u1, float& v1)
{
  if(!ancestor.valid() || !descendant.valid())
    return false;
  const int steps = int(descendant.level()) - int(ancestor.level());
  if(steps <= 0 || steps > 20)   // >20 cannot happen: GGGS has 21 levels
    return false;
  const std::uint64_t span = std::uint64_t(1) << steps;
  const std::uint64_t base_row = std::uint64_t(ancestor.row()) * span;
  const std::uint64_t base_col = std::uint64_t(ancestor.column()) * span;
  const std::uint64_t row = descendant.row();
  const std::uint64_t col = descendant.column();
  if(row < base_row || row - base_row >= span || col < base_col || col - base_col >= span)
    return false;
  const double inv = 1.0 / double(span);
  const std::uint64_t row_off = row - base_row;
  const std::uint64_t col_off = col - base_col;
  u0 = float(double(col_off) * inv);
  u1 = float(double(col_off + 1) * inv);
  v0 = float(double(span - 1 - row_off) * inv);   // rows go north, v goes south
  v1 = float(double(span - row_off) * inv);
  return true;
}

// [camp#172] Off-thread reload worker: load each evicted fine tile's cached GeoTIFF
// from the fine-tile cache dir. Takes everything by value so it is fully
// self-contained — safe even if the layer is destroyed before it finishes (same
// contract as writeTileToCache). Skips any index whose file is missing/unreadable;
// the caller clears every attempted index regardless (onReloadFinished).
//
// Threading-model divergence from GggsTileLayer (deliberate): that loader supports
// abort of an in-flight batch so a subsequent pan can cancel stale work; this worker
// returns its tiles by value and is consumed via future().result() with NO abort. A
// kick therefore always runs to completion — a subsequent pan cannot cancel it. Two
// guards keep that cheap rather than costly: kickReload bounds each batch to a
// quarter-budget of tiles (so an uncancellable kick is small), and the reload_attempted_
// gate blocks overlapping kicks. Abort was not worth the extra machinery for batches this
// bounded; revisit if the per-kick cap ever needs to grow.
std::vector<SonarLiveTile> reloadTilesFromCache(std::vector<gggs::GridIndex> indices,
                                                std::string dir, std::uint8_t level_value)
{
  // [#217] First statement of a QtConcurrent worker: give this pool thread an
  // alternate signal stack, without which a stack-overflow SIGSEGV here cannot
  // be reported at all. Idempotent — a thread_local pointer test — so the pool
  // pays for it once per thread, not once per task. Guarded by the
  // check_worker_alt_stacks test; see camp_crash/crash_handler.h.
  camp_crash::install_thread_alt_stack();

  namespace fs = std::filesystem;
  const gggs::Level level(level_value);
  std::vector<SonarLiveTile> loaded;
  loaded.reserve(indices.size());
  for(const gggs::GridIndex& index : indices)
  {
    const std::string path = (fs::path(dir) / (tileStem(index) + ".tif")).string();
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
  // [#217] First statement of a QtConcurrent worker: give this pool thread an
  // alternate signal stack, without which a stack-overflow SIGSEGV here cannot
  // be reported at all. Idempotent — a thread_local pointer test — so the pool
  // pays for it once per thread, not once per task. Guarded by the
  // check_worker_alt_stacks test; see camp_crash/crash_handler.h.
  camp_crash::install_thread_alt_stack();

  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(dir, ec);
  if(ec)
    return;   // unwritable cache dir — skip the GDAL round-trip (best-effort cache)
  const std::string final_path = (fs::path(dir) / (tileStem(tile.index()) + ".tif")).string();
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

rclcpp::QoS catalogSubscriptionQos()
{
  // Complete snapshot, latest wins: reliable, depth 1.
  //
  // [field 2026-08-27] Durability must be VOLATILE, not transient-local. The boat-side
  // producer IS transient-local, but camp runs on the OPERATOR side and never
  // sees that publisher: it receives the catalog as republished by
  // `udp_bridge`, which emits VOLATILE. A transient-local subscriber is
  // QoS-incompatible with a volatile publisher, so the subscription never
  // matched and handleCatalog() never fired — the anti-entropy reconcile
  // (ADR-0006 D4) never pruned, and stale coverage persisted on screen
  // indefinitely (verified live on pandy: publisher /operator/udp_bridge
  // RELIABLE/VOLATILE vs subscriber /operator/camp RELIABLE/TRANSIENT_LOCAL).
  //
  // Tradeoff, accepted: volatile means there is no latched sample, so camp no
  // longer gets the current catalog immediately on join. It must wait for the
  // next publication of the catalog before it can reconcile. A never-matching
  // subscription delivers nothing at all, so waiting strictly dominates.
  rclcpp::QoS qos(1);
  qos.durability(rclcpp::DurabilityPolicy::Volatile).reliable();
  return qos;
}

void SonarLiveCacheLayer::subscribeCatalog()
{
  if(catalog_sub_ || !node_ || !node_->node())
    return;
  const rclcpp::QoS qos = catalogSubscriptionQos();
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
  // requests) fire on every enable — the sample may have arrived while
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
  // [camp#171 triage] Synchronously consume the completed reload via onReloadFinished()
  // (as waitForReload() does) rather than a bare reload_attempted_.clear(): the
  // QFutureWatcher::finished delivery stays queued after waitForFinished(), so consuming
  // it here — which clears reload_attempted_ — makes that queued slot no-op on the
  // double-invoke guard (reload_attempted_.empty()). A bare clear instead leaves the
  // queued finished to bind result() to a NEW in-flight future after a re-enable + kick,
  // blocking the GUI thread and dropping that batch. onReloadFinished() is a no-op when
  // no reload was in flight (reload_attempted_ empty), so this is safe unconditionally.
  reload_watcher_.waitForFinished();
  onReloadFinished();
  evicted_fine_indices_.clear();
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

  // [camp#169] Always buffer, even while disabled. The catalog is published only
  // when it changes, and the boat's catalog is stable for long stretches, so a
  // sample discarded here while disabled may not be re-delivered for the rest of
  // the session — no reconcile would ever fire. enableLiveCoverage() replays this
  // buffer. [field 2026-08-27] The volatile subscription makes this MORE load-bearing, not
  // less: there is no latched sample to re-deliver on a late join either, so this
  // buffer is now the only path from a catalog seen while disabled to a reconcile
  // on enable. The catalog subscription itself is never torn down by
  // disableLiveCoverage() (only the tile stream is), so samples keep arriving and
  // landing here throughout a disabled window.
  last_catalog_ = msg;

  // [field 2026-08-27 / ADR-0010 D5] Record what the boat says EXISTS. This is the
  // set a coarse placeholder may be drawn for (planDraw), so it is maintained from
  // every catalog — including one that arrives while disabled, exactly like the
  // camp#169 buffer above, since a re-enable replays that buffer rather than
  // waiting for a catalog that may not be republished for the rest of the session.
  // A complete catalog is a full snapshot, so this is an assignment, not a merge:
  // a tile that left the catalog must stop being placeholder-able at once.
  catalogued_fine_.clear();
  for(const auto& entry : msg.entries)
  {
    const gggs::GridIndex index = gridIndexFromTileIndex(entry.index);
    if(index.valid())
      catalogued_fine_.insert(index);
  }

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
    // Delete the disk copy too (best-effort). [field 2026-08-27] Drop the queued
    // write-through first, or a coalesced relaunch would re-create the file we are
    // about to remove (see cancelPendingWrite for the in-flight case).
    cancelPendingWrite(index);
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::remove(fineTilePath(cache_dir_, index), ec);
    // [camp#172] The disk copy is gone, so this index is no longer reloadable — drop
    // it from the on-demand reload set, or kickReload() would snapshot it and
    // loadFromGeoTiff() would fail forever with the index never cleared.
    evicted_fine_indices_.erase(index);
    reconciler_.drop(index);
  }
  if(gl_current)
    renderer_.doneCurrent();

  // [field 2026-08-27] Propagate prune-on-absence into the DERIVED overview pyramid
  // (ADR-0010 D7). Pruning only `tiles_` leaves the folded coarse copy of a retracted
  // region on screen and on disk forever — it is warm-loaded back on every restart, so
  // no code path could ever reflect a boat-side store reset (measured on pandy: fine
  // tiles pruned 107M -> 44M while 41M of `overviews/` survived and kept the pre-reset
  // coverage in front of the operator).
  const bool pyramid_changed = reconcilePyramid(catalog, result.to_prune);
  if(pyramid_changed)
    evictIfOverBudget();   // a rebuild can pull a non-resident overview back into memory

  if(pruned || pyramid_changed)
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

void SonarLiveCacheLayer::cancelPendingWrite(const gggs::GridIndex& index)
{
  // [field 2026-08-27] GUI thread. The tile is being deleted, so drop its queued
  // write-through — without this a coalesced relaunch would re-create the `.tif` the
  // caller is about to remove, and warm-load would resurrect the deleted tile on the
  // next restart.
  //
  // An IN-FLIGHT write cannot be cancelled, and its WriteState must NOT be erased:
  // scheduleWriteThrough() keys "is a worker already running for this index?" on that
  // state, so erasing it would let a later fold launch a SECOND concurrent worker
  // racing the first on the shared `<stem>.tif.tmp` path — the exact bug the
  // coalescing exists to prevent. Clear `dirty` instead and let
  // onWriteThroughFinished() erase the state. The already-running worker may still
  // land its file after the caller's remove(); the next catalog's pyramid sweep scans
  // the directory and deletes it again, so that window self-heals rather than
  // persisting.
  auto it = write_states_.find(index);
  if(it == write_states_.end())
    return;
  it->second.dirty = false;
  if(!it->second.in_flight)
    write_states_.erase(it);
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
  // added to the reconciler (ADR-0010 D4: they are a local derived product and are not
  // in the boat's catalog, so reconciling them directly would prune them all on the
  // first catalog).
  //
  // [field 2026-08-27] That is a statement about the RECONCILER's membership, not an
  // exemption from prune-on-absence, and this comment used to assert the latter. It was
  // wrong, and the bug it described as intent shipped: a retracted region kept its
  // coarse coverage forever. The pyramid is derived from the fine tiles, so the catalog
  // is authoritative over it too — see reconcilePyramid(), which removes overviews with
  // no catalogued descendant and REBUILDS the ones that merely lost some. The rebuild is
  // the other half of the reason: this fold is purely accumulative (foldChild only ever
  // folds data IN — there is no un-fold), so a withdrawn contribution can never be
  // subtracted from a parent in place.
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

// ------------------- pyramid prune-on-absence (ADR-0010 D7) ------------------

bool SonarLiveCacheLayer::reconcilePyramid(
  const marine_tiled_raster_store::TileCatalog& catalog,
  const std::vector<gggs::GridIndex>& pruned_fine)
{
  // [field 2026-08-27] Converge the DERIVED overview pyramid to the catalog, the same
  // way handleCatalog() converges the fine tiles. The pyramid is a function of the fine
  // tiles, so the catalog is authoritative over it too — an overview tile SHOULD exist
  // exactly when it is an ancestor of a live fine index.
  //
  // Two distinct failure modes, two distinct repairs:
  //   * stale by absence   — no live descendant at all: the whole tile is stale, remove
  //                          it (memory + GPU texture + disk).
  //   * dirty by partial   — some descendants live, but a descendant was withdrawn:
  //     withdrawal            foldChild() has no inverse, so the withdrawn contribution
  //                          cannot be subtracted in place. Rebuild the tile from its
  //                          surviving children instead of keeping it.
  //
  // Deliberately NOT done: invalidating the whole ancestor chain of every pruned tile.
  // That chain always reaches level 0, which spans a whole GGGS grid at the coarsest
  // level, so any single ordinary retraction would destroy the apex — and tiles are
  // withdrawn and re-added in normal operation. The work here is proportionate to what
  // actually went away: nothing is removed while a live descendant remains, and only
  // ancestors of something that really was withdrawn are rebuilt.
  namespace fs = std::filesystem;
  std::error_code ec;

  // Prune-gate parity with the reconciler (ADR-0008 D4b): a generation_time of 0 — an
  // un-stamped or sim-time-0 catalog — disables prune-on-absence there, because no held
  // version can be strictly older than 0. It must disable it here too, or an unstamped
  // catalog would sweep the entire pyramid while every fine tile stayed put.
  if(catalog.generation_time == 0)
    return false;

  // ---- 1. The authoritative live-ancestor set --------------------------------
  // Ancestors of every catalogued fine index, unioned with the ancestors of the fine
  // tiles we still possess locally. The second half is what keeps this consistent with
  // the reconciler's per-tile timestamp gate: a held fine tile that is absent from the
  // catalog but too NEW to prune is still on disk and still legitimately folded into
  // its ancestors, so those ancestors must not be swept out from under it. Walking each
  // chain only until it meets an already-recorded ancestor keeps this linear in the
  // size of the result, not catalog-size x pyramid-depth.
  std::set<gggs::GridIndex> live;
  auto addChain = [&live](const gggs::GridIndex& index)
  {
    for(gggs::GridIndex a = gggs::parent(index); a.valid(); a = gggs::parent(a))
      if(!live.insert(a).second)
        break;
  };
  for(const auto& entry : catalog.entries)
    addChain(entry.index);
  for(const auto& item : tiles_)                              // post-prune residency
    addChain(item.first);
  for(const gggs::GridIndex& index : evicted_fine_indices_)   // post-prune, on disk
    addChain(index);

  // ---- 2. Stale by absence ---------------------------------------------------
  std::set<gggs::GridIndex> gone;
  for(const auto& item : overview_tiles_)
    if(!live.count(item.first))
      gone.insert(item.first);

  // The `overviews/` sub-dir must be swept too, not just the resident map. Memory-
  // pressure eviction (evictIfOverBudget phase 2) frees an overview Entry but KEEPS its
  // disk copy on purpose — the disk cache is the durable backing (ADR-0010 D2) — so a
  // stale overview can be invisible in overview_tiles_ and still be warm-loaded back on
  // the next restart. That is precisely how the pre-reset coverage kept returning on
  // pandy. Collect first, delete after: removing entries under a live
  // directory_iterator is not defined.
  std::vector<fs::path> stale_files;
  for(fs::directory_iterator it(fs::path(cache_dir_) / kOverviewSubdir, ec), end;
      !ec && it != end; it.increment(ec))
  {
    if(it->path().extension() != ".tif")
      continue;
    const gggs::GridIndex index = indexFromStem(it->path().stem().string());
    if(!index.valid() || live.count(index))
      continue;
    stale_files.push_back(it->path());
    gone.insert(index);
  }
  // Delete the stale files BEFORE the rebuild below reads children off disk, so a stale
  // child can never be folded back into the parent that is being repaired.
  for(const fs::path& path : stale_files)
    fs::remove(path, ec);

  // ---- 3. Dirty by partial withdrawal ----------------------------------------
  // A surviving ancestor of anything withdrawn this pass holds folded contributions
  // that can no longer be subtracted. Ancestors that were themselves removed above are
  // skipped (nothing to rebuild), and a chain stops as soon as it meets an ancestor
  // already queued — everything above that is queued too.
  std::set<gggs::GridIndex> dirty;
  auto markChainDirty = [&](const gggs::GridIndex& index)
  {
    for(gggs::GridIndex a = gggs::parent(index); a.valid(); a = gggs::parent(a))
    {
      if(!live.count(a))
        continue;
      if(!dirty.insert(a).second)
        break;
    }
  };
  for(const gggs::GridIndex& index : pruned_fine)
    markChainDirty(index);
  for(const gggs::GridIndex& index : gone)
    markChainDirty(index);

  // Only rebuild overviews that actually exist (resident or on disk) — the pyramid is
  // built by folding, never by this repair path, so fabricating a tile here would both
  // invent coverage and do unbounded work. Deepest level first, so a parent's rebuild
  // sees its children already repaired.
  std::vector<gggs::GridIndex> targets;
  for(const gggs::GridIndex& index : dirty)
    if(overview_tiles_.count(index) ||
       fs::exists(overviewTilePath(cache_dir_, index), ec))
      targets.push_back(index);
  std::sort(targets.begin(), targets.end(),
            [](const gggs::GridIndex& a, const gggs::GridIndex& b)
            { return a.level() > b.level(); });

  std::map<gggs::GridIndex, SonarLiveTile> staged;
  const std::size_t stale_count = gone.size();
  for(const gggs::GridIndex& index : targets)
  {
    std::optional<SonarLiveTile> rebuilt = rebuiltOverview(index, staged, gone);
    if(rebuilt)
      staged.emplace(index, std::move(*rebuilt));
    else
      gone.insert(index);   // no surviving child: an honest gap beats a stale tile
  }

  // ---- 4. Apply --------------------------------------------------------------
  // [camp#134] A removed Entry owns a QOpenGLTexture whose dtor frees GPU resources
  // ONLY under a current GL context; erasing it without one leaks the texture. Same
  // hasContext()/makeCurrent()/doneCurrent() discipline as the fine prune above and as
  // the destructor's teardown.
  const bool gl_current = !gone.empty() && renderer_.hasContext() && renderer_.makeCurrent();
  for(const gggs::GridIndex& index : gone)
  {
    auto it = overview_tiles_.find(index);
    if(it != overview_tiles_.end())
    {
      it->second.texture.reset();
      overview_tiles_.erase(it);
    }
    cancelPendingWrite(index);
  }
  if(gl_current)
    renderer_.doneCurrent();
  // Every removed tile's disk copy. The stale-by-absence files were already deleted
  // above (before the rebuild read children off disk, so a stale child could not be
  // folded back in) — repeating them here is a harmless no-op and keeps this the single
  // place that guarantees "in `gone` => no file left behind", which is what the
  // unrebuildable tiles need.
  for(const gggs::GridIndex& index : gone)
    if(!staged.count(index))
      fs::remove(overviewTilePath(cache_dir_, index), ec);

  for(auto& item : staged)
  {
    auto it = overview_tiles_.find(item.first);
    if(it == overview_tiles_.end())
      it = overview_tiles_
             .emplace(item.first, Entry{std::move(item.second), nullptr, true, 0})
             .first;
    else
    {
      // Mutate the Entry IN PLACE. Replacing it would destroy the old
      // QOpenGLTexture here, outside any GL context (this runs after doneCurrent());
      // keeping the texture object and marking it dirty lets textureFor() re-upload it
      // under the renderer's context at draw time, which is where every other
      // re-upload happens.
      it->second.tile = std::move(item.second);
      it->second.texture_dirty = true;
    }
    // Persist the repair, or warm-load would bring the stale content straight back.
    scheduleWriteThrough(it->second.tile, kOverviewSubdir);
  }

  const bool changed = !gone.empty() || !staged.empty();
  if(changed)
    qInfo().noquote() << "[live coverage" << QString::fromStdString(base_namespace_)
                      << "] pyramid reconcile: removed" << qulonglong(stale_count)
                      << "overview tiles with no live descendant and"
                      << qulonglong(gone.size() - stale_count)
                      << "with no rebuildable children; rebuilt"
                      << qulonglong(staged.size()) << "that lost a descendant";
  return changed;
}

std::optional<SonarLiveTile> SonarLiveCacheLayer::rebuiltOverview(
  const gggs::GridIndex& index,
  const std::map<gggs::GridIndex, SonarLiveTile>& staged,
  const std::set<gggs::GridIndex>& gone) const
{
  // [field 2026-08-27] Rebuild ONE overview tile from scratch out of its surviving
  // children. foldChild() only ever folds data IN — there is no un-fold — so a tile
  // that lost a descendant cannot have that contribution subtracted; starting from an
  // empty parent and re-folding what is left is the only correct repair.
  //
  // This reproduces exactly how the tile was built in the first place: foldIntoParent()
  // folds the WHOLE parent tile into the grandparent, so every pyramid level above the
  // fine tiles is by construction "the fold of my children". Cells no surviving child
  // covers stay NoData and are discarded by the renderer.
  //
  // std::nullopt means "no child survives" — the caller then removes the tile. A
  // zoomed-out gap is honest; a knowingly-stale tile is not.
  if(!level_)
    return std::nullopt;   // fine level unknown: cannot tell a fine child from a coarse one
  std::optional<SonarLiveTile> parent;
  for(const gggs::GridIndex& kid : gggs::children(index))
  {
    const std::optional<SonarLiveTile> child = overviewRebuildChild(kid, staged, gone);
    if(!child)
      continue;
    if(!parent)
      parent.emplace(index, child->width(), child->height());   // foldChild needs same size
    parent->foldChild(*child);
  }
  return parent;
}

std::optional<SonarLiveTile> SonarLiveCacheLayer::overviewRebuildChild(
  const gggs::GridIndex& child,
  const std::map<gggs::GridIndex, SonarLiveTile>& staged,
  const std::set<gggs::GridIndex>& gone) const
{
  // [field 2026-08-27] Freshest copy of one rebuild input, or nullopt when it is gone.
  // Order: a child already repaired in this same pass (targets run deepest-first) beats
  // the resident copy, which beats the disk copy. A child this pass withdrew
  // contributes nothing — that omission IS the repair.
  if(gone.count(child))
    return std::nullopt;
  const auto staged_it = staged.find(child);
  if(staged_it != staged.end())
    return staged_it->second;

  std::error_code ec;
  const bool child_is_fine = (child.level() == *level_);
  if(child.level() > *level_)
    return std::nullopt;   // below the finest level this source publishes: no such tile
  const std::map<gggs::GridIndex, Entry>& pool = child_is_fine ? tiles_ : overview_tiles_;
  const auto resident = pool.find(child);
  if(resident != pool.end())
    return resident->second.tile;
  // Not resident: the disk cache is the durable backing for BOTH pools (ADR-0010 D2),
  // so an evicted child is still a valid input. exists() first so a missing file is not
  // a GDAL open failure on the log.
  const std::filesystem::path path = child_is_fine ? fineTilePath(cache_dir_, child)
                                                   : overviewTilePath(cache_dir_, child);
  if(!std::filesystem::exists(path, ec))
    return std::nullopt;
  return SonarLiveTile::loadFromGeoTiff(path.string(), gggs::Level(child.level()));
}

const SonarLiveTile* SonarLiveCacheLayer::overviewTileForTest(
  const gggs::GridIndex& index) const
{
  const auto it = overview_tiles_.find(index);
  return it == overview_tiles_.end() ? nullptr : &it->second.tile;
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
      // [camp#172] Remember a fine tile's resident footprint so kickReload can bound how
      // many it reloads at once (captured before the texture is freed, so it reflects the
      // full displayed cost a reload will re-incur once painted).
      last_evicted_fine_bytes_ = entryBytes(it->second);
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
  // directly (indexSceneRect — literally the clip test planDraw() applies). A null
  // viewport (headless with no clip) matches nothing — reload is viewport-driven.
  // [field 2026-08-27] Consistent with the draw set by construction: every index here is
  // in pendingFineIndices() (evicted, so not resident), i.e. every tile this wants to
  // reload is one currently shown as a coarse placeholder, and a completed reload
  // replaces that placeholder with the real thing. The converse does not hold and must
  // not: a catalogued tile we have never received is placeholder-able but has no disk
  // copy to reload — it is a REQUEST candidate (the reconciler's job), not a reload one.
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
  //
  // The `reload_attempted_` guard is load-bearing, not just an optimization: the
  // watcher's `finished` signal is queued, so `isRunning()` flips false one event-loop
  // hop BEFORE onReloadFinished() runs and consumes the result. A paint() in that
  // window would otherwise pass the `!isRunning()` gate and `setFuture()` a second
  // future — the pending onReloadFinished() would then read the NEW future's result()
  // (possibly blocking the GUI thread), drop the first reload, and overwrite the
  // attempted set. `reload_attempted_` is non-empty for exactly that window (kickReload
  // sets it; onReloadFinished clears it), so gating on it empty closes the race.
  if(reload_watcher_.isRunning() || !reload_attempted_.empty() || !level_ ||
     viewport_scene.isNull())
    return;
  std::vector<gggs::GridIndex> visible;
  for(const gggs::GridIndex& index : evicted_fine_indices_)
    if(indexSceneRect(index).intersects(viewport_scene))
      visible.push_back(index);
  if(visible.empty())
    return;

  // [camp#172 / ADR-0010 D6] Bound the per-kick reload volume. onReloadFinished() inserts
  // the whole batch BEFORE evictIfOverBudget() runs, so an unbounded kick (a wide
  // zoom-out re-entering the whole survey) would spike residency far over budget in one
  // shot. Cap the batch to a quarter-budget's worth of fine tiles — the same headroom the
  // hysteresis gate guarantees before a kick — keeping the nearest-to-viewport-centre
  // ones (the rest re-kick on the next paint once these are consumed and the viewport
  // moves). last_evicted_fine_bytes_ is non-zero whenever there is anything to reload
  // (eviction set it), so this only no-ops the cap when the budget is disabled.
  if(vram_budget_bytes_ > 0 && last_evicted_fine_bytes_ > 0)
  {
    const std::size_t cap_bytes =
      static_cast<std::size_t>(vram_budget_bytes_ * (1.0 - kReloadHysteresisFactor));
    const std::size_t max_tiles = std::max<std::size_t>(1, cap_bytes / last_evicted_fine_bytes_);
    if(visible.size() > max_tiles)
    {
      const QPointF centre = viewport_scene.center();
      std::sort(visible.begin(), visible.end(),
                [&](const gggs::GridIndex& a, const gggs::GridIndex& b)
                {
                  const QPointF ca = indexSceneRect(a).center();
                  const QPointF cb = indexSceneRect(b).center();
                  const double da = (ca.x() - centre.x()) * (ca.x() - centre.x()) +
                                    (ca.y() - centre.y()) * (ca.y() - centre.y());
                  const double db = (cb.x() - centre.x()) * (cb.x() - centre.x()) +
                                    (cb.y() - centre.y()) * (cb.y() - centre.y());
                  return da < db;
                });
      qInfo().noquote() << "[live coverage" << QString::fromStdString(base_namespace_)
                        << "] reload capped to" << qulonglong(max_tiles) << "of"
                        << qulonglong(visible.size())
                        << "visible evicted tiles (nearest first); rest reload on later frames";
      visible.resize(max_tiles);
    }
  }

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
    // Defensive: possession was kept across eviction (D2 never drops markHave), so the
    // reconciler already holds this index — but re-assert it so residency and possession
    // can never silently diverge if that invariant is ever weakened. markHave is
    // idempotent (newest version wins).
    reconciler_.markHave(index, tile.version());
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
     reload_attempted_.empty() &&
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

std::set<gggs::GridIndex> SonarLiveCacheLayer::pendingFineIndices() const
{
  // [field 2026-08-27 / ADR-0010 D5] The fine tiles KNOWN TO EXIST that are not
  // resident. Coarse overview data may be drawn for exactly these indices and for
  // nothing else, so this set is the whole licence for painting anything coarse.
  //
  // Two authorities are unioned because neither alone is complete:
  //   * the catalog is authoritative for what exists on the boat, but it is empty
  //     until the first one arrives (a warm start with the link down never sees
  //     one, yet the disk cache is full of real coverage);
  //   * evicted_fine_indices_ is locally known — an evicted tile whose disk copy
  //     survives — but it is CLEARED after a reload attempt whether or not the
  //     load succeeded (camp#172, so a permanently unloadable index cannot re-kick
  //     forever), which would otherwise silently blank a region the catalog still
  //     lists.
  // A resident index is excluded: it draws itself at full resolution, and a
  // placeholder under it would be a coarse wash bleeding past its edges.
  std::set<gggs::GridIndex> pending;
  for(const gggs::GridIndex& index : catalogued_fine_)
    if(!tiles_.count(index))
      pending.insert(index);
  for(const gggs::GridIndex& index : evicted_fine_indices_)
    if(!tiles_.count(index))
      pending.insert(index);
  return pending;
}

const SonarLiveCacheLayer::Entry* SonarLiveCacheLayer::finestResidentAncestor(
  const gggs::GridIndex& index, gggs::GridIndex* source) const
{
  // [field 2026-08-27] Walk up the GGGS chain and stop at the first resident
  // overview: the finest coarse representation this pyramid holds for @p index.
  // Nothing resident anywhere up the chain means the pyramid has no data over it,
  // so the caller draws nothing — an honest gap beats inventing coverage.
  for(gggs::GridIndex a = gggs::parent(index); a.valid(); a = gggs::parent(a))
  {
    auto it = overview_tiles_.find(a);
    if(it != overview_tiles_.end())
    {
      if(source)
        *source = a;
      return &it->second;
    }
  }
  return nullptr;
}

std::vector<SonarLiveCacheLayer::DrawItem> SonarLiveCacheLayer::planDraw(
  const QRectF& clip_scene) const
{
  // [field 2026-08-27 / ADR-0010 D5] Resolve the draw list. Two contributions, and
  // deliberately no third:
  //
  //   1. a COARSE PLACEHOLDER for every fine tile that is known to exist but has
  //      not loaded (pendingFineIndices()), sampled out of its finest resident
  //      ancestor and clipped — via the texture sub-rect — to that fine tile's own
  //      footprint;
  //   2. every RESIDENT fine tile, whole, on top.
  //
  // What is NOT drawn is the point of this function. An overview tile used to be
  // drawn over its own extent, which is 4^n fine tiles wide: at level 0 a single
  // cell is ~667 x 926 m of the mean of everything folded beneath it, painted
  // across water nobody has ever surveyed and over the chart underneath. The
  // premise that licensed it — "where a fine tile is present it fully covers its
  // parent" (camp#160) — is false: a child covers a QUARTER of its parent, so the
  // parent showed through everywhere the finer level was sparse, which is most of
  // a survey in progress. Measured on pandy during the BizzyBoat deployment: the
  // operator saw a solid ~650 m block over a good part of the survey area.
  //
  // Scale does not appear here at all. A placeholder exists exactly while a real
  // tile is missing, so the zoomed-out case needs no separate threshold: zoom out
  // over a whole survey and nearly every catalogued tile is non-resident, so the
  // union of their footprints IS the coverage (ADR-0010 D1).
  //
  // [camp#103] A non-null @p clip_scene culls by Web-Mercator extent here, before
  // any texture is touched, so an offscreen tile is neither uploaded nor drawn.
  std::vector<DrawItem> plan;
  const bool clipped = !clip_scene.isNull();

  for(const gggs::GridIndex& index : pendingFineIndices())
  {
    if(clipped && !indexSceneRect(index).intersects(clip_scene))
      continue;
    gggs::GridIndex source;
    const Entry* ancestor = finestResidentAncestor(index, &source);
    if(!ancestor)
      continue;
    DrawItem item;
    item.footprint = index;
    item.source = source;
    item.placeholder = true;
    if(!textureWindowOf(source, index, item.u0, item.v0, item.u1, item.v1))
      continue;   // not a real descendant (see textureWindowOf) — draw nothing
    plan.push_back(item);
  }

  for(const auto& entry : tiles_)
  {
    if(clipped && !indexSceneRect(entry.first).intersects(clip_scene))
      continue;
    DrawItem item;
    item.footprint = entry.first;
    item.source = entry.first;
    plan.push_back(item);
  }
  return plan;
}

QList<raster::RasterFieldItem> SonarLiveCacheLayer::items()
{
  return itemsIntersecting(QRectF());
}

QList<raster::RasterFieldItem> SonarLiveCacheLayer::itemsIntersecting(
  const QRectF& clip_scene)
{
  // [camp#134] Turn planDraw()'s selection into Scalar items for the shared
  // renderer. Called with the renderer's GL context current (renderImage()), so
  // textureFor() may lazily (re)upload here. The geo->Web-Mercator warp lives in
  // the renderer; this only forwards each footprint's lat/lon extent, the sampled
  // texture sub-rect, and the source tile's NoData sentinel.
  // [camp#103] The clip test already happened in planDraw(), i.e. BEFORE
  // textureFor(), so an offscreen tile is still neither uploaded nor drawn.
  const std::vector<DrawItem> plan = planDraw(clip_scene);
  QList<raster::RasterFieldItem> result;
  result.reserve(int(plan.size()));
  for(const DrawItem& planned : plan)
  {
    auto& pool = planned.placeholder ? overview_tiles_ : tiles_;
    auto it = pool.find(planned.source);
    if(it == pool.end())
      continue;
    Entry& entry = it->second;
    const SonarLiveBand* band = entry.tile.band(band_name_);
    if(!band)
      continue;
    QOpenGLTexture* texture = textureFor(entry);
    if(!texture)
      continue;
    raster::RasterFieldItem fi;
    fi.texture = texture;
    fi.format = raster::RasterFieldItem::Format::Scalar;
    fi.geographic = true;
    // The extent painted is the FOOTPRINT's, not the source tile's: for a coarse
    // placeholder they differ, and the sub-rect below is what keeps the coarse
    // texture clipped to it instead of being squashed across it.
    fi.west = planned.footprint.westLongitude();
    fi.east = planned.footprint.eastLongitude();
    fi.south = planned.footprint.southLatitude();
    fi.north = planned.footprint.northLatitude();
    fi.u0 = planned.u0;
    fi.v0 = planned.v0;
    fi.u1 = planned.u1;
    fi.v1 = planned.v1;
    fi.has_nodata = band->has_nodata;
    fi.nodata = band->has_nodata ? band->nodata : 0.0f;
    result.push_back(fi);
  }
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
  // [camp#103] Only what intersects the clip contributes: the resident fine tiles
  // plus a coarse placeholder for each known-but-not-loaded one (ADR-0010 D5).
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
  // re-kicking every frame — mirrors GggsTileLayer's demand-driven loader. The
  // `reload_attempted_.empty()` guard closes the queued-`finished` race (see kickReload):
  // don't kick a second reload while a finished one's result is still unconsumed.
  if(!reload_watcher_.isRunning() &&
     reload_attempted_.empty() &&
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
