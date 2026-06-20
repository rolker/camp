#include "gggs_tile_layer.h"

#include "gggs_tile.h"
#include "../map_view/web_mercator.h"

#include <QAction>
#include <QColor>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QGeoCoordinate>
#include <QMenu>
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
  releaseGL();
}

void GggsTileLayer::loadDirectory(const QString& directory)
{
  QDir dir(directory);
  const QStringList files = dir.entryList(QStringList() << "*.tif" << "*.tiff",
                                          QDir::Files, QDir::Name);
  bool first_extent = true;   // first geometrically-valid tile (scene_bounds_)
  bool first_range = true;    // first tile WITH valid samples (data range)
  for(const QString& name : files)
  {
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

    // Data range tracks the first tile that actually HAS samples — separately
    // from the extent, or an all-NoData first tile would leave data_min_ stuck
    // at the sentinel (its default 1.0 is never < a floored-to->=1 sample).
    if(tile->dataMin() <= tile->dataMax())
    {
      if(first_range || tile->dataMin() < data_min_) data_min_ = tile->dataMin();
      if(first_range || tile->dataMax() > data_max_) data_max_ = tile->dataMax();
      first_range = false;
    }
    tiles_.push_back(std::move(tile));
  }
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
  if(tiles_.empty() || data_min_ > data_max_)
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

void GggsTileLayer::contextMenu(QMenu* menu)
{
  map::Layer::contextMenu(menu);
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(auto type : map::ColorMap::allTypes())
  {
    QAction* action = colormap_menu->addAction(map::ColorMap::name(type));
    action->setCheckable(true);
    action->setChecked(type == colormap_.type());
    connect(action, &QAction::triggered, this, [this, type]() { setColormap(type); });
  }
}

void GggsTileLayer::readSettings()
{
  map::Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  const map::ColorMap::Type type = map::ColorMap::typeFromName(
    settings.value("colormap", map::ColorMap::name(colormap_.type())).toString());
  settings.endGroup();
  settings.endGroup();
  if(type != colormap_.type())
  {
    colormap_.setType(type);
    lut_dirty_ = true;
    cached_image_ = QImage();
  }
}

void GggsTileLayer::writeSettings()
{
  map::Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  settings.setValue("colormap", map::ColorMap::name(colormap_.type()));
  settings.endGroup();
  settings.endGroup();
}

}  // namespace raster
}  // namespace camp
