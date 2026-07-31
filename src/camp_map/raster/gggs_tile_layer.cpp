#include "gggs_tile_layer.h"

#include "gggs_tile.h"
#include "gggs_tile_util.h"
#include "colormap_range_dialog.h"
#include "viewport_clip.h"
#include "../map_view/web_mercator.h"

#include <marine_colormap/palette.hpp>

#include <QAction>
#include <QDir>
#include <QFileInfo>
#include <QGeoCoordinate>
#include <QMenu>
#include <QSet>
#include <QSettings>
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
  // [camp#102] tilesReady() folds completed tiles' ranges + repaints on the GUI
  // thread when the async pixel load finishes.
  connect(&future_watcher_, &QFutureWatcher<void>::finished, this,
          &GggsTileLayer::tilesReady);
  loadDirectory(directory);
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
  else
    setStatus("(no tiles)");
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
  QDir dir(directory);
  const QStringList files = dir.entryList(QStringList() << "*.tif" << "*.tiff",
                                          QDir::Files, QDir::Name);
  bool first_extent = true;   // first geometrically-valid tile (scene_bounds_)
  for(const QString& name : files)
  {
    // [camp#112] Skip companion tiles (`_time`/`_source`): the `*.tif` glob also
    // matches them, but only the base value tile is renderable.
    if(!isValueTile(name))
      continue;
    // [camp#102] Extent/metadata only — the GggsTile ctor no longer reads pixels.
    // boundingRect()/sceneBounds() are valid immediately (fit-to-extent works at
    // load time); the band reads (and therefore the data range) are deferred to
    // the async loadTiles() worker, so data_min_/data_max_ accumulate
    // incrementally in tilesReady() rather than here.
    auto tile = std::make_unique<GggsTile>(dir.filePath(name));
    if(!tile->valid())
      continue;

    // Tile extent in Web-Mercator scene units. geoToMap is monotonic in both
    // lon and lat, so the two opposite geographic corners give the scene rect.
    const QPointF lo = web_mercator::geoToMap(
      QGeoCoordinate(tile->minLat(), tile->minLon()));
    const QPointF hi = web_mercator::geoToMap(
      QGeoCoordinate(tile->maxLat(), tile->maxLon()));
    const QRectF tile_rect = QRectF(lo, hi).normalized();
    scene_bounds_ = first_extent ? tile_rect : scene_bounds_.united(tile_rect);
    first_extent = false;

    tiles_.push_back(std::move(tile));
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
  QSet<QString> known;
  for(const auto& tile : tiles_)
    known.insert(tile->path());

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
    new_tiles.push_back(std::move(tile));
  }

  if(new_tiles.empty())
    return false;   // nothing new — leave any in-flight load running untouched

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
    const bool first_extent = tiles_.empty() && scene_bounds_.isNull();
    const QPointF lo = web_mercator::geoToMap(
      QGeoCoordinate(tile->minLat(), tile->minLon()));
    const QPointF hi = web_mercator::geoToMap(
      QGeoCoordinate(tile->maxLat(), tile->maxLon()));
    const QRectF tile_rect = QRectF(lo, hi).normalized();
    prepareGeometryChange();
    scene_bounds_ = first_extent ? tile_rect : scene_bounds_.united(tile_rect);
    if(first_extent)
    {
      setTransform(QTransform::fromScale(1.0, -1.0));
      setPos(QPointF(scene_bounds_.left(), scene_bounds_.bottom()));
    }
    tiles_.push_back(std::move(tile));
  }

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

  setStatus("(loading...)");
  future_watcher_.setFuture(QtConcurrent::run(this, &GggsTileLayer::loadTilesWorker));
}

void GggsTileLayer::loadTilesWorker()
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
  for(auto& tile : tiles_)
  {
    {
      QMutexLocker lock(&abort_flag_mutex_);
      if(abort_flag_)
        return;
    }
    if(!tile->pixelsLoaded())
      tile->loadPixels();
  }
}

void GggsTileLayer::tilesReady()
{
  // [camp#102] GUI thread, after the worker's join. Fold each loaded tile's range
  // into the layer auto-range incrementally (the range is unknown until a tile's
  // pixels load — an all-NoData tile reports a crossed range and is skipped).
  bool first_range = (data_min_ > data_max_);
  for(auto& tile : tiles_)
  {
    // [camp#108] Only fold tiles that actually carry the layer's current band.
    // A non-uniform tile-set leaves a tile lacking that band on its prior band
    // (applyBand() keeps it loaded rather than blanking it); its stale prior-band
    // min/max must not pollute the current band's auto-range.
    if(tile->band() != band_)
      continue;
    if(!tile->pixelsLoaded() || tile->dataMin() > tile->dataMax())
      continue;
    if(first_range || tile->dataMin() < data_min_) data_min_ = tile->dataMin();
    if(first_range || tile->dataMax() > data_max_) data_max_ = tile->dataMax();
    first_range = false;
  }
  cached_image_ = QImage();   // re-render now that pixels (and the range) exist
  // [camp#102] If the range is still crossed after the fold, every loaded tile was
  // all-NoData (or failed to read): there is nothing to draw and clearing the
  // status would leave a silently-blank enabled layer. Signal "(no data)" so the
  // operator can tell an empty tile-set from one that simply hasn't loaded yet.
  if(data_min_ > data_max_)
    setStatus("(no data)");
  else
    setStatus("");
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
  if(!load_started_)
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
  for(auto& tile : tiles_)
  {
    // [camp#102] Skip a tile whose pixels haven't loaded yet. pixelsLoaded() is an
    // ACQUIRE load pairing with the worker's RELEASE store in loadPixels(), so once
    // true the data_/texture() reads see the worker's completed writes — no race.
    if(!tile->pixelsLoaded())
      continue;
    if(!clip_scene.isNull())
    {
      const QPointF lo = web_mercator::geoToMap(
        QGeoCoordinate(tile->minLat(), tile->minLon()));
      const QPointF hi = web_mercator::geoToMap(
        QGeoCoordinate(tile->maxLat(), tile->maxLon()));
      if(!QRectF(lo, hi).normalized().intersects(clip_scene))
        continue;
    }
    QOpenGLTexture* texture = tile->texture();
    if(!texture)
      continue;
    RasterFieldItem item;
    item.texture = texture;
    item.format = RasterFieldItem::Format::Scalar;
    item.geographic = true;
    item.west = tile->minLon();
    item.east = tile->maxLon();
    item.south = tile->minLat();
    item.north = tile->maxLat();
    // [camp#122] Per-tile NoData: a non-uniform store can carry different NoData
    // per tile/band, so it travels per item rather than as one layer uniform.
    item.has_nodata = tile->hasNoData();
    item.nodata = tile->hasNoData() ? float(tile->noData()) : 0.0f;
    result.push_back(item);
  }
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

  // [camp#102] Lazily kick the async pixel load on the first paint — QGraphicsView
  // only paints visible items, so this defers band reads to layers the operator
  // turns on (default-off tile-sets never load). Kick exactly once; tilesReady()
  // folds the range + repaints when the worker finishes. Until then the range is
  // still crossed and there is nothing to draw, so fall through and return.
  if(!load_started_)
  {
    load_started_ = true;
    loadTiles();
  }

  if(data_min_ > data_max_)   // no tile's pixels/range folded yet
    return;

  // [camp#103 / ADR-0011] Render only the viewport-visible clip of the extent,
  // sized to its on-screen pixels — a zoomed-in view of a store far larger than
  // kMaxImageEdge stays crisp. Re-render when the size (zoom) OR the clip (pan)
  // changes; a viewport-sized FBO makes the per-frame pan re-render cheap.
  const ViewportClip clip =
    deriveViewportClip(painter, this, boundingRect(), scene_bounds_, kMaxImageEdge);

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
    showColormapRangeDialog(
      nullptr, "Colormap range", state,
      [this](float lo, float hi) { setRangeOverride(lo, hi); },
      [this]() { resetRangeToAuto(); });
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
