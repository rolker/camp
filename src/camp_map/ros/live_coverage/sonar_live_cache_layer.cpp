#include "sonar_live_cache_layer.h"

#include "../node.h"
#include "../../map_view/web_mercator.h"

#include <marine_colormap/palette.hpp>

#include <QAction>
#include <QDebug>
#include <QDir>
#include <QGeoCoordinate>
#include <QInputDialog>
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

// Off-thread write-through worker. Takes everything by value so it is fully
// self-contained — safe even if the layer is destroyed before it finishes.
// Atomic temp+rename for crash safety only (ADR-0006 D2).
void writeTileToCache(SonarLiveTile tile, std::string dir)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  fs::create_directories(dir, ec);
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
  // [camp#134] Release each tile's GL texture under the renderer's context (the
  // textures are owned by the Entries, not the renderer). The renderer frees its
  // own program/LUT/FBO/context in its destructor right after this.
  if(renderer_.makeCurrent())
  {
    for(auto& item : tiles_)
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

  // [camp#134] insert_or_assign below replaces any pre-existing Entry for an index
  // (reachable on a disable→re-enable: disable leaves tiles_ — and their GL textures
  // — intact). A displaced Entry's QOpenGLTexture must be freed under a current GL
  // context, so make it current for the seed loop when tiles already hold textures.
  // On the first (empty-map) warm load there is nothing to displace, so skip it.
  // Gate on hasContext() (mirrors GggsTileLayer::applyBand): when no context exists
  // yet no texture was ever uploaded, so skip makeCurrent() rather than forcing the
  // offscreen context into existence to reset null textures.
  const bool gl_current =
    !tiles_.empty() && renderer_.hasContext() && renderer_.makeCurrent();
  const gggs::Level level(*level_);
  for(auto& tile : SonarLiveTile::loadCacheDir(cache_dir_, level))
  {
    const gggs::GridIndex index = tile.index();
    if(!index.valid())
      continue;
    // Seed the reconciler at version 0: "have something" — older than any real
    // catalog version, so the next catalog re-requests it if the boat is newer,
    // and the prune gate never deletes it spuriously (ADR-0006 D3).
    reconciler_.markHave(index, 0);
    tiles_.insert_or_assign(index, Entry{std::move(tile), nullptr, true});
  }
  if(gl_current)
    renderer_.doneCurrent();
  if(band_name_.empty())
    band_name_ = defaultBand();
  recomputeBounds();
  resetAutoRange();
  foldAutoRange();
  updateDisplay();
}

// ------------------------------ message handlers -----------------------------

void SonarLiveCacheLayer::handleTile(const marine_interfaces::msg::SonarVisualizationTile& msg)
{
  if(!enabled_)
    return;
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
                        Entry{SonarLiveTile(index, msg.width, msg.height), nullptr, true})
           .first;
  Entry& entry = it->second;
  entry.tile.applyPatch(msg);
  entry.texture_dirty = true;
  reconciler_.markHave(index, entry.tile.version());
  // TODO(camp): eviction-by-area follow-up — only prune-on-absence bounds the
  // cache today, so a long survey can accumulate many tiles (ADR-0006 consequences).

  scheduleWriteThrough(entry.tile);

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

void SonarLiveCacheLayer::scheduleWriteThrough(const SonarLiveTile& tile)
{
  // GUI thread. Coalesce per tile: snapshot the latest state and launch a worker
  // only if this tile is idle. If a worker is already in flight for this tile, mark
  // it dirty — onWriteThroughFinished() re-launches once with the latest snapshot,
  // so two workers never race on the shared <stem>.tif.tmp path (the prior bug),
  // and intermediate patches are coalesced into a single follow-up write.
  WriteState& ws = write_states_[tile.index()];
  ws.pending = tile;   // latest wins
  ws.dirty = true;
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
  watcher->setFuture(QtConcurrent::run(writeTileToCache, std::move(*ws.pending), cache_dir_));
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

// ------------------------------ extent / range -------------------------------

void SonarLiveCacheLayer::recomputeBounds()
{
  prepareGeometryChange();
  QRectF bounds;
  bool first = true;
  for(const auto& entry : tiles_)
  {
    const SonarLiveTile& tile = entry.second.tile;
    const QPointF lo = web_mercator::geoToMap(QGeoCoordinate(tile.minLat(), tile.minLon()));
    const QPointF hi = web_mercator::geoToMap(QGeoCoordinate(tile.maxLat(), tile.maxLon()));
    const QRectF rect = QRectF(lo, hi).normalized();
    bounds = first ? rect : bounds.united(rect);
    first = false;
  }
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
  for(const auto& entry : tiles_)
  {
    const SonarLiveBand* band = entry.second.tile.band(band_name_);
    if(!band || band->data_min > band->data_max)
      continue;
    if(first || band->data_min < data_min_) data_min_ = band->data_min;
    if(first || band->data_max > data_max_) data_max_ = band->data_max;
    first = false;
  }
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
    setStatus(QString("(live: %1 tiles)").arg(qulonglong(tiles_.size())));
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
  // [camp#134] Collect the held tiles' selected band as Scalar items for the shared
  // renderer. Called with the renderer's GL context current (renderImage()), so
  // textureFor() may lazily (re)upload here. The geo->Web-Mercator warp lives in
  // the renderer; this only forwards each tile's lat/lon extent + NoData sentinel.
  QList<raster::RasterFieldItem> result;
  result.reserve(int(tiles_.size()));
  for(auto& item : tiles_)
  {
    Entry& entry = item.second;
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
    fi.west = entry.tile.minLon();
    fi.east = entry.tile.maxLon();
    fi.south = entry.tile.minLat();
    fi.north = entry.tile.maxLat();
    fi.has_nodata = band->has_nodata;
    fi.nodata = band->has_nodata ? band->nodata : 0.0f;
    result.push_back(fi);
  }
  return result;
}

QImage SonarLiveCacheLayer::renderImage(const QSize& size)
{
  if(tiles_.empty() || data_min_ > data_max_ || size.isEmpty())
    return QImage();
  // [camp#134] Make the renderer's context current, collect the held tiles
  // (uploading textures under it), then delegate the warp + draw.
  if(!renderer_.makeCurrent())
    return QImage();
  const QList<raster::RasterFieldItem> draw = items();
  // [camp#142] Feed the resolved range (Auto tracks data_min_/data_max_; Manual is
  // the operator override) into the shader's u_min/u_max instead of the raw extents.
  const QImage image = renderer_.renderToImage(draw, scene_bounds_, range_model_.lo(),
                                               range_model_.hi(), size);
  renderer_.doneCurrent();
  return image;
}

void SonarLiveCacheLayer::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*)
{
  if(tiles_.empty() || data_min_ > data_max_)
    return;

  const QRectF dev = painter->worldTransform().mapRect(boundingRect());
  const int w = std::min(kMaxImageEdge, std::max(1, int(std::ceil(std::abs(dev.width())))));
  const int h = std::min(kMaxImageEdge, std::max(1, int(std::ceil(std::abs(dev.height())))));
  const QSize size(w, h);

  if(cached_image_.isNull() || cached_size_ != size)
  {
    cached_image_ = renderImage(size);
    cached_size_ = size;
  }
  if(cached_image_.isNull())
    return;

  painter->save();
  painter->setRenderHint(QPainter::SmoothPixmapTransform);
  painter->drawImage(boundingRect(), cached_image_);
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

  // [camp#141] Expose the FULL marine_colormap registry, not just the legacy ramps.
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(const std::string& name : marine_colormap::palette_names())
  {
    QAction* action = colormap_menu->addAction(QString::fromStdString(name));
    action->setCheckable(true);
    action->setChecked(name == renderer_.colormap());
    connect(action, &QAction::triggered, this, [this, name]() { setColormap(name); });
  }

  // [camp#142] Colormap range override. Live bands are scalar values (same gating as
  // the Colormap submenu above), so the submenu is offered unconditionally. "Set
  // range…" prompts for lo then hi (pre-filled with the current resolved range) and
  // pins a Manual override; "Reset to auto" returns to the data-driven extents.
  QMenu* range_menu = menu->addMenu("Colormap range");
  QAction* set_range = range_menu->addAction("Set range…");
  connect(set_range, &QAction::triggered, this, [this]()
  {
    bool ok = false;
    const double lo = QInputDialog::getDouble(
      nullptr, "Colormap range", "Minimum:", range_model_.lo(),
      -1.0e9, 1.0e9, 6, &ok);
    if(!ok)
      return;
    const double hi = QInputDialog::getDouble(
      nullptr, "Colormap range", "Maximum:", range_model_.hi(),
      -1.0e9, 1.0e9, 6, &ok);
    if(!ok)
      return;
    setRangeOverride(float(lo), float(hi));
  });
  QAction* reset_range = range_menu->addAction("Reset to auto");
  connect(reset_range, &QAction::triggered, this, [this]() { resetRangeToAuto(); });

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

void SonarLiveCacheLayer::readSettings()
{
  Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(settingsKey());
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
