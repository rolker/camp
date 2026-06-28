#include "sonar_live_cache_layer.h"

#include "../node.h"
#include "../../map_view/web_mercator.h"

#include <QAction>
#include <QColor>
#include <QDebug>
#include <QDir>
#include <QGeoCoordinate>
#include <QMatrix4x4>
#include <QMenu>
#include <QMetaObject>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
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

// NOTE: shader duplicated from GggsTileLayer; will unify via RasterFieldSource in
// camp#134. The pipeline is identical (CPU-side geo->Web-Mercator warp per vertex,
// single-band R32F value texture, colormap LUT, per-tile NoData discard); only the
// data source differs (in-memory dequantized Float32 vs. GggsTile's GDAL read).
constexpr char kVertexShader[] = R"(
#version 120
attribute vec2 a_pos;
attribute vec2 a_texcoord;
uniform mat4 u_mvp;
varying vec2 v_texcoord;
void main()
{
  gl_Position = u_mvp * vec4(a_pos, 0.0, 1.0);
  v_texcoord = a_texcoord;
}
)";

constexpr char kFragmentShader[] = R"(
#version 120
uniform sampler2D u_tex;
uniform sampler2D u_lut;
uniform float u_min;
uniform float u_max;
uniform int u_has_nodata;
uniform float u_nodata;
varying vec2 v_texcoord;
void main()
{
  float v = texture2D(u_tex, v_texcoord).r;
  if(u_has_nodata != 0 && v == u_nodata)
    discard;
  float t = clamp((v - u_min) / max(u_max - u_min, 1.0), 0.0, 1.0);
  vec4 c = texture2D(u_lut, vec2(t, 0.5));
  gl_FragColor = vec4(c.rgb, 1.0);
}
)";

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
  // Join any in-flight write-through (each worker is self-contained, but joining
  // matches the camp_map worker-lifetime convention). Then tear down GL.
  write_watcher_.waitForFinished();
  releaseGL();
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

  bool pruned = false;
  for(const auto& index : result.to_prune)
  {
    auto it = tiles_.find(index);
    if(it != tiles_.end())
    {
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
  if(pruned)
  {
    recomputeBounds();
    foldAutoRange();
    cached_image_ = QImage();
    update(boundingRect());
  }
  updateDisplay();
}

void SonarLiveCacheLayer::writeThroughFinished()
{
  // Hook kept for symmetry; the worker is self-contained, so nothing to fold.
}

void SonarLiveCacheLayer::scheduleWriteThrough(const SonarLiveTile& tile)
{
  // Copy the tile + dir by value into a self-contained worker (no `this` deref).
  write_watcher_.setFuture(QtConcurrent::run(writeTileToCache, tile, cache_dir_));
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

bool SonarLiveCacheLayer::ensureGL()
{
  if(gl_failed_)
    return false;
  if(gl_context_)
    return true;
  gl_surface_ = new QOffscreenSurface();
  gl_surface_->create();
  gl_context_ = new QOpenGLContext();
  if(!gl_surface_->isValid() || !gl_context_->create())
  {
    qWarning("SonarLiveCacheLayer: offscreen GL unavailable; tiles not rendered");
    gl_failed_ = true;
    delete gl_context_; gl_context_ = nullptr;
    delete gl_surface_; gl_surface_ = nullptr;
    return false;
  }
  return true;
}

bool SonarLiveCacheLayer::ensureProgram()
{
  if(program_)
    return program_->isLinked();
  program_ = std::make_unique<QOpenGLShaderProgram>();
  program_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader);
  program_->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader);
  if(!program_->link())
  {
    qWarning("SonarLiveCacheLayer: shader link failed: %s",
             program_->log().toUtf8().constData());
    setStatus("(shader error)");
    return false;
  }
  return true;
}

QOpenGLTexture* SonarLiveCacheLayer::ensureLut()
{
  if(lut_texture_ && !lut_dirty_)
    return lut_texture_.get();
  std::vector<uchar> lut(256 * 4);
  for(int i = 0; i < 256; ++i)
  {
    const QColor c = colormap_.colorNormalized(i / 255.0);
    lut[i * 4 + 0] = uchar(c.red());
    lut[i * 4 + 1] = uchar(c.green());
    lut[i * 4 + 2] = uchar(c.blue());
    lut[i * 4 + 3] = uchar(c.alpha());
  }
  if(!lut_texture_)
  {
    lut_texture_ = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
    lut_texture_->setFormat(QOpenGLTexture::RGBA8_UNorm);
    lut_texture_->setSize(256, 1);
    lut_texture_->setMipLevels(1);
    lut_texture_->allocateStorage(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8);
    lut_texture_->setMinMagFilters(QOpenGLTexture::Linear, QOpenGLTexture::Linear);
    lut_texture_->setWrapMode(QOpenGLTexture::ClampToEdge);
  }
  lut_texture_->setData(QOpenGLTexture::RGBA, QOpenGLTexture::UInt8, lut.data());
  lut_dirty_ = false;
  return lut_texture_.get();
}

QOpenGLTexture* SonarLiveCacheLayer::textureFor(Entry& entry)
{
  const SonarLiveBand* band = entry.tile.band(band_name_);
  if(!band || band->data.empty())
    return nullptr;
  if(entry.texture && !entry.texture_dirty)
    return entry.texture.get();
  // (Re)upload the selected band as an R32F value texture, Nearest-filtered so the
  // shader's exact-equality NoData discard never blends across a sentinel boundary
  // (same rationale as GggsTile::texture()).
  entry.texture = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
  entry.texture->setFormat(QOpenGLTexture::R32F);
  entry.texture->setSize(entry.tile.width(), entry.tile.height());
  entry.texture->setMipLevels(1);
  entry.texture->allocateStorage(QOpenGLTexture::Red, QOpenGLTexture::Float32);
  entry.texture->setData(QOpenGLTexture::Red, QOpenGLTexture::Float32, band->data.data());
  entry.texture->setMinMagFilters(QOpenGLTexture::Nearest, QOpenGLTexture::Nearest);
  entry.texture->setWrapMode(QOpenGLTexture::ClampToEdge);
  entry.texture_dirty = false;
  return entry.texture.get();
}

QImage SonarLiveCacheLayer::renderImage(const QSize& size)
{
  if(tiles_.empty() || data_min_ > data_max_ || size.isEmpty())
    return QImage();
  if(!ensureGL())
    return QImage();
  if(!gl_context_->makeCurrent(gl_surface_))
  {
    qWarning("SonarLiveCacheLayer: makeCurrent failed; tiles not rendered");
    gl_failed_ = true;
    return QImage();
  }

  QOpenGLFunctions* f = gl_context_->functions();
  if(!fbo_ || fbo_->size() != size)
    fbo_ = std::make_unique<QOpenGLFramebufferObject>(size);

  fbo_->bind();
  f->glViewport(0, 0, size.width(), size.height());
  f->glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  f->glClear(GL_COLOR_BUFFER_BIT);
  f->glDisable(GL_DEPTH_TEST);
  f->glEnable(GL_BLEND);
  f->glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

  if(ensureProgram())
  {
    const double origin_x = scene_bounds_.left();
    const double origin_y = scene_bounds_.top();
    const double width_m = scene_bounds_.width();
    const double height_m = scene_bounds_.height();
    QMatrix4x4 mvp;
    mvp.ortho(0.0f, float(width_m), 0.0f, float(height_m), -1.0f, 1.0f);

    program_->bind();
    program_->setUniformValue("u_mvp", mvp);
    program_->setUniformValue("u_min", float(data_min_));
    program_->setUniformValue("u_max", float(data_max_));
    program_->setUniformValue("u_tex", 0);
    program_->setUniformValue("u_lut", 1);
    QOpenGLTexture* lut = ensureLut();
    if(lut)
      lut->bind(1);

    const int pos_loc = program_->attributeLocation("a_pos");
    const int texcoord_loc = program_->attributeLocation("a_texcoord");
    program_->enableAttributeArray(pos_loc);
    program_->enableAttributeArray(texcoord_loc);

    const int rows = kLatSubdivisions + 1;
    std::vector<float> verts;
    verts.reserve(rows * 2 * 4);
    for(auto& item : tiles_)
    {
      Entry& entry = item.second;
      const SonarLiveBand* band = entry.tile.band(band_name_);
      if(!band)
        continue;
      QOpenGLTexture* texture = textureFor(entry);
      if(!texture)
        continue;

      verts.clear();
      const double min_lon = entry.tile.minLon();
      const double max_lon = entry.tile.maxLon();
      const double min_lat = entry.tile.minLat();
      const double max_lat = entry.tile.maxLat();
      for(int r = 0; r < rows; ++r)
      {
        const double frac = double(r) / kLatSubdivisions;
        const double lat = max_lat + (min_lat - max_lat) * frac;   // north -> south
        const float v = float(frac);
        const QPointF l = web_mercator::geoToMap(QGeoCoordinate(lat, min_lon));
        const QPointF rt = web_mercator::geoToMap(QGeoCoordinate(lat, max_lon));
        verts.insert(verts.end(),
                     {float(l.x() - origin_x), float(l.y() - origin_y), 0.0f, v});
        verts.insert(verts.end(),
                     {float(rt.x() - origin_x), float(rt.y() - origin_y), 1.0f, v});
      }

      texture->bind(0);
      program_->setUniformValue("u_has_nodata", band->has_nodata ? 1 : 0);
      program_->setUniformValue("u_nodata", band->has_nodata ? band->nodata : 0.0f);
      program_->setAttributeArray(pos_loc, GL_FLOAT, verts.data(), 2, 4 * sizeof(float));
      program_->setAttributeArray(texcoord_loc, GL_FLOAT, verts.data() + 2, 2,
                                  4 * sizeof(float));
      f->glDrawArrays(GL_TRIANGLE_STRIP, 0, rows * 2);
      texture->release(0);
    }

    if(lut)
      lut->release(1);
    program_->disableAttributeArray(pos_loc);
    program_->disableAttributeArray(texcoord_loc);
    program_->release();
  }

  fbo_->release();
  QImage image = fbo_->toImage();
  gl_context_->doneCurrent();
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

void SonarLiveCacheLayer::releaseGL()
{
  if(gl_context_ && gl_surface_ && gl_context_->makeCurrent(gl_surface_))
  {
    fbo_.reset();
    program_.reset();
    lut_texture_.reset();
    for(auto& item : tiles_)
      item.second.texture.reset();
    gl_context_->doneCurrent();
  }
  delete gl_context_; gl_context_ = nullptr;
  delete gl_surface_; gl_surface_ = nullptr;
}

// ------------------------------- band / colormap -----------------------------

void SonarLiveCacheLayer::setColormap(map::ColorMap::Type type)
{
  if(type == colormap_.type())
    return;
  colormap_.setType(type);
  lut_dirty_ = true;
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

  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(auto type : map::ColorMap::allTypes())
  {
    QAction* action = colormap_menu->addAction(map::ColorMap::name(type));
    action->setCheckable(true);
    action->setChecked(type == colormap_.type());
    connect(action, &QAction::triggered, this, [this, type]() { setColormap(type); });
  }

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
  const map::ColorMap::Type type = map::ColorMap::typeFromName(
    settings.value("colormap", map::ColorMap::name(colormap_.type())).toString());
  const std::string band = settings.value("band", QString::fromStdString(band_name_))
                             .toString().toStdString();
  const bool was_enabled = settings.value("live_enabled", false).toBool();
  settings.endGroup();
  settings.endGroup();

  if(type != colormap_.type())
  {
    colormap_.setType(type);
    lut_dirty_ = true;
    cached_image_ = QImage();
  }
  if(!band.empty())
    band_name_ = band;
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
  settings.setValue("colormap", map::ColorMap::name(colormap_.type()));
  settings.setValue("band", QString::fromStdString(band_name_));
  settings.setValue("live_enabled", enabled_);
  settings.endGroup();
  settings.endGroup();
}

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp
