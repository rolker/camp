#include "gggs_tile_layer.h"

#include "gggs_tile.h"
#include "gggs_tile_util.h"
#include "../map_view/web_mercator.h"

#include <QAction>
#include <QColor>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QGeoCoordinate>
#include <QMenu>
#include <QSet>
#include <QSettings>
#include <QMatrix4x4>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QPainter>
#include <QTransform>
#include <QtConcurrent>

#include <cmath>
#include <vector>

namespace camp
{
namespace raster
{

namespace
{

// Vertex shader: purely linear. The geo->Web-Mercator warp is done on the CPU in
// double precision (web_mercator::geoToMap) per mesh vertex, with positions made
// RELATIVE to the extent origin so values stay small. This is deliberate:
// computing y = R*asinh(tan phi) in the shader used GPU transcendentals (tan/log/
// sqrt) whose low precision, multiplied by R~6.4e6, produced a ~50 m latitude
// error (longitude was exact because x = R*lambda needs no transcendental).
constexpr char kVertexShader[] = R"(
#version 120
attribute vec2 a_pos;        // local Web-Mercator metres (from extent origin)
attribute vec2 a_texcoord;
uniform mat4 u_mvp;          // local metres -> NDC
varying vec2 v_texcoord;
void main()
{
  gl_Position = u_mvp * vec4(a_pos, 0.0, 1.0);
  v_texcoord = a_texcoord;
}
)";

// Fragment shader: auto-ranged value mapped through a colormap LUT (the shared
// camp::map::ColorMap baked to a 256x1 RGBA texture on the CPU). NoData (0,
// reserved by the mosaicker; real returns floored to >= 1) is discarded so empty
// cells are transparent. Premultiplied-alpha output (opaque, so straight == premult).
//
// [camp#108] LIMITATION: the `v <= 0.0` discard assumes the mosaicker's
// floor-to-1 convention (true for the depth/sidescan band 1). A switched band
// whose valid samples can be 0 or negative (e.g. an uncertainty or signed-offset
// band) will have those samples discarded and the rest mis-ranged. Distinguishing
// real NoData from valid 0/negative samples needs per-band NoData plumbed to the
// shader (a uniform + a sentinel test) — deferred to the camp#122 follow-up (see
// also plan Open Questions).
constexpr char kFragmentShader[] = R"(
#version 120
uniform sampler2D u_tex;     // unit 0: single-band data (R32F)
uniform sampler2D u_lut;     // unit 1: colormap LUT (256x1 RGBA)
uniform float u_min;
uniform float u_max;
varying vec2 v_texcoord;
void main()
{
  float v = texture2D(u_tex, v_texcoord).r;
  if(v <= 0.0)
    discard;
  float t = clamp((v - u_min) / max(u_max - u_min, 1.0), 0.0, 1.0);
  vec4 c = texture2D(u_lut, vec2(t, 0.5));
  gl_FragColor = vec4(c.rgb, 1.0);
}
)";

}  // namespace

GggsTileLayer::GggsTileLayer(map::MapItem* parentItem, const QString& directory):
  map::Layer(parentItem, QFileInfo(directory).fileName()),
  directory_(directory)
{
  // [camp#102] tilesReady() folds completed tiles' ranges + repaints on the GUI
  // thread when the async pixel load finishes.
  connect(&future_watcher_, &QFutureWatcher<void>::finished, this,
          &GggsTileLayer::tilesReady);
  loadDirectory(directory);
  if(!tiles_.empty())
  {
    // Match the camp2 raster convention (RasterLayer / MapTiles / grids): a
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
  releaseGL();
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

bool GggsTileLayer::ensureGL()
{
  // gl_failed_ first: once GL is declared broken (creation OR a mid-session
  // makeCurrent failure), stay failed and don't retry — otherwise every repaint
  // re-enters renderImage (cached_image_ never populates) and re-warns.
  if(gl_failed_)
    return false;
  if(gl_context_)
    return true;

  gl_surface_ = new QOffscreenSurface();
  gl_surface_->create();
  gl_context_ = new QOpenGLContext();
  if(!gl_surface_->isValid() || !gl_context_->create())
  {
    qWarning("GggsTileLayer: offscreen GL unavailable; tiles not rendered");
    gl_failed_ = true;
    delete gl_context_; gl_context_ = nullptr;
    delete gl_surface_; gl_surface_ = nullptr;
    return false;
  }
  return true;
}

bool GggsTileLayer::ensureProgram()
{
  if(program_)
    return program_->isLinked();
  program_ = std::make_unique<QOpenGLShaderProgram>();
  program_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader);
  program_->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader);
  if(!program_->link())
  {
    qWarning("GggsTileLayer: shader link failed: %s",
             program_->log().toUtf8().constData());
    setStatus("(shader error)");
    return false;
  }
  return true;
}

QOpenGLTexture* GggsTileLayer::ensureLut()
{
  // Bake camp::map::ColorMap into a 256x1 RGBA LUT (re-baked when the ramp
  // changes). Sampled by the fragment shader as the colour transfer.
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

QImage GggsTileLayer::renderImage(const QSize& size)
{
  if(tiles_.empty() || data_min_ > data_max_ || size.isEmpty())
    return QImage();
  if(!ensureGL())
    return QImage();
  if(!gl_context_->makeCurrent(gl_surface_))
  {
    qWarning("GggsTileLayer: makeCurrent failed; tiles not rendered");
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
    // Vertices are LOCAL Web-Mercator metres from the extent origin (the SW
    // corner), so map [0, width] x [0, height] -> NDC. local-y 0 = south,
    // height = north, so 'top' = height gives a NORTH-UP image (row 0 = north);
    // the item's fromScale(1,-1) + NW anchor draws it upright.
    const double origin_x = scene_bounds_.left();    // west
    const double origin_y = scene_bounds_.top();      // south (smaller mercator-y)
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

    // Per-tile triangle strip, subdivided in latitude only. Each vertex's
    // Web-Mercator position is computed on the CPU (double precision) via
    // web_mercator::geoToMap and made relative to the extent origin. Interleaved
    // [local_x, local_y, u, v] per vertex.
    const int rows = kLatSubdivisions + 1;
    std::vector<float> verts;
    verts.reserve(rows * 2 * 4);
    for(auto& tile : tiles_)
    {
      // [camp#102] Skip a tile whose pixels haven't loaded yet (in-flight or not
      // yet kicked). pixelsLoaded() is an ACQUIRE load that pairs with the worker's
      // RELEASE store in loadPixels() (issued after the data_ move), so once it
      // reads true the subsequent data_/texture() reads are guaranteed to see the
      // worker's completed writes — no race even while a worker is mid-flight on
      // another tile. The layer also repaints on tilesReady() once the load
      // completes (to fold the range).
      if(!tile->pixelsLoaded())
        continue;

      verts.clear();
      const double min_lon = tile->minLon();
      const double max_lon = tile->maxLon();
      const double min_lat = tile->minLat();
      const double max_lat = tile->maxLat();
      for(int r = 0; r < rows; ++r)
      {
        const double frac = double(r) / kLatSubdivisions;
        const double lat = max_lat + (min_lat - max_lat) * frac;   // north -> south
        const float v = float(frac);                               // tex row 0 = north
        const QPointF l = web_mercator::geoToMap(QGeoCoordinate(lat, min_lon));
        const QPointF rt = web_mercator::geoToMap(QGeoCoordinate(lat, max_lon));
        verts.insert(verts.end(),
                     {float(l.x() - origin_x), float(l.y() - origin_y), 0.0f, v});
        verts.insert(verts.end(),
                     {float(rt.x() - origin_x), float(rt.y() - origin_y), 1.0f, v});
      }

      QOpenGLTexture* texture = tile->texture();
      if(!texture)
        continue;
      texture->bind(0);
      program_->setAttributeArray(pos_loc, GL_FLOAT, verts.data(), 2,
                                  4 * sizeof(float));
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
  QImage image = fbo_->toImage();   // top-down ARGB32 (premultiplied)
  gl_context_->doneCurrent();
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

  // Target the offscreen render at the extent's on-screen size, so the image is
  // crisp at the current zoom. Re-render only when that size changes (zoom);
  // pan reuses the cached image (drawImage repositions it via the world xform).
  const QRectF dev = painter->worldTransform().mapRect(boundingRect());
  const int w = std::min(kMaxImageEdge,
                         std::max(1, int(std::ceil(std::abs(dev.width())))));
  const int h = std::min(kMaxImageEdge,
                         std::max(1, int(std::ceil(std::abs(dev.height())))));
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

void GggsTileLayer::releaseGL()
{
  if(gl_context_ && gl_surface_ && gl_context_->makeCurrent(gl_surface_))
  {
    fbo_.reset();
    program_.reset();
    lut_texture_.reset();
    for(auto& tile : tiles_)
      tile->releaseGL();
    gl_context_->doneCurrent();
  }
  delete gl_context_; gl_context_ = nullptr;
  delete gl_surface_; gl_surface_ = nullptr;
}

void GggsTileLayer::setColormap(map::ColorMap::Type type)
{
  if(type == colormap_.type())
    return;
  colormap_.setType(type);
  lut_dirty_ = true;
  cached_image_ = QImage();   // force a re-render with the new ramp
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
  // the switch just clears pixels + reloads with no stale GL state. But if the
  // context EXISTS and makeCurrent FAILS, the old-band textures can't be released;
  // setBand() below still clears each tile's pixels, and texture() never refreshes
  // an EXISTING texture, so a stale old-band texture would render against the new
  // band's range once pixels reload. Match renderImage()'s response to a
  // makeCurrent failure — mark GL failed so ensureGL() short-circuits and the layer
  // stops rendering rather than drawing a stale-band frame.
  bool have_context = false;
  if(gl_context_ && gl_surface_)
  {
    if(gl_context_->makeCurrent(gl_surface_))
      have_context = true;
    else
    {
      qWarning("GggsTileLayer: makeCurrent failed during band switch; "
               "tiles not rendered");
      gl_failed_ = true;
    }
  }
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
    gl_context_->doneCurrent();
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

  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(auto type : map::ColorMap::allTypes())
  {
    QAction* action = colormap_menu->addAction(map::ColorMap::name(type));
    action->setCheckable(true);
    action->setChecked(type == colormap_.type());
    connect(action, &QAction::triggered, this, [this, type]() { setColormap(type); });
  }

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

void GggsTileLayer::readSettings()
{
  map::Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  // [camp#102] Default GGGS tile-set leaves OFF: re-read `visible` with a FALSE
  // fallback (Layer::readSettings just applied it with a TRUE default). This has
  // to live in the leaf override, not a ctor setVisible(false) — MapItem::
  // itemConstructed() runs readSettings() via QTimer::singleShot(0,...) AFTER the
  // ctor, which would clobber a ctor call (map_item.cpp:26,180-183). A persisted
  // `visible` value still wins (the operator's on/off choice round-trips); only
  // the first-run default flips.
  setVisible(settings.value("visible", false).toBool());
  const map::ColorMap::Type type = map::ColorMap::typeFromName(
    settings.value("colormap", map::ColorMap::name(colormap_.type())).toString());
  // [camp#108] Persisted band (default 1). Applied via applyBand() below — the
  // non-persisting band switch (texture release + reload + range reset) — so the
  // read path does NOT write the value straight back out (setBand() would). Only
  // when it differs, to skip the abort+reload on the common no-change path,
  // mirroring the inline colormap apply directly below.
  const int band = settings.value("band", 1).toInt();
  settings.endGroup();
  settings.endGroup();
  if(type != colormap_.type())
  {
    colormap_.setType(type);
    lut_dirty_ = true;
    cached_image_ = QImage();
  }
  if(band != band_)
    applyBand(band);
}

void GggsTileLayer::writeSettings()
{
  map::Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  settings.setValue("colormap", map::ColorMap::name(colormap_.type()));
  settings.setValue("band", band_);   // [camp#108] selected band round-trips
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
