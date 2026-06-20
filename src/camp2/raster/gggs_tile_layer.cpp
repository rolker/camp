#include "gggs_tile_layer.h"

#include "gggs_tile.h"
#include "../map_view/web_mercator.h"

#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QGeoCoordinate>
#include <QMatrix4x4>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QPainter>

#include <cmath>
#include <vector>

namespace camp
{
namespace raster
{

namespace
{

// Vertex shader: per-vertex geo->Web-Mercator warp. Line-by-line port of
// web_mercator::geoToMap (x = R*lambda; y = R*asinh(tan phi)); asinh expanded as
// log(t + sqrt(t^2+1)) to stay valid on GLSL profiles without the asinh builtin.
// The geoToMap-parity unit test pins the formula so the shader can't drift.
constexpr char kVertexShader[] = R"(
#version 120
attribute vec2 a_lonlat;     // degrees
attribute vec2 a_texcoord;
uniform mat4 u_mvp;          // scene(Web-Mercator) -> NDC
uniform float u_radius;      // earth_radius_at_equator
varying vec2 v_texcoord;
void main()
{
  float deg2rad = 0.0174532925199432958;
  float x = a_lonlat.x * deg2rad * u_radius;
  float t = tan(a_lonlat.y * deg2rad);
  float y = log(t + sqrt(t * t + 1.0)) * u_radius;   // asinh(t) * R
  gl_Position = u_mvp * vec4(x, y, 0.0, 1.0);
  v_texcoord = a_texcoord;
}
)";

// Fragment shader: auto-ranged grayscale of the single-band value. NoData (0,
// reserved by the mosaicker; real returns floored to >= 1) is discarded so empty
// cells are transparent. Premultiplied-alpha output. Band-select + real colormap
// are Slice 3 (camp#63).
constexpr char kFragmentShader[] = R"(
#version 120
uniform sampler2D u_tex;
uniform float u_min;
uniform float u_max;
varying vec2 v_texcoord;
void main()
{
  float v = texture2D(u_tex, v_texcoord).r;
  if(v <= 0.0)
    discard;
  float g = clamp((v - u_min) / max(u_max - u_min, 1.0), 0.0, 1.0);
  gl_FragColor = vec4(g, g, g, 1.0);
}
)";

}  // namespace

GggsTileLayer::GggsTileLayer(map::MapItem* parentItem, const QString& directory):
  map::Layer(parentItem, QFileInfo(directory).fileName()),
  directory_(directory)
{
  loadDirectory(directory);
  if(!tiles_.empty())
    // Position the item at the extent's corner and paint in small LOCAL
    // coordinates. QGraphicsView/QPainter lose precision rasterizing at raw
    // Web-Mercator magnitudes (~1e7), which offsets the image; RasterLayer
    // avoids this the same way (setPos + local pixel space).
    setPos(scene_bounds_.topLeft());
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
  bool first = true;
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
    scene_bounds_ = first ? tile_rect : scene_bounds_.united(tile_rect);

    if(tile->dataMin() <= tile->dataMax())   // tile has valid samples
    {
      if(first || tile->dataMin() < data_min_) data_min_ = tile->dataMin();
      if(first || tile->dataMax() > data_max_) data_max_ = tile->dataMax();
    }
    first = false;
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
  if(gl_context_)
    return true;
  if(gl_failed_)
    return false;

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
    // Map the layer's Web-Mercator extent to NDC. scene_bounds_ is normalised,
    // so top() is the smaller mercator-y (south) and bottom() the larger
    // (north). We want the offscreen QImage's row 0 to be the SOUTH edge, so
    // that drawImage(boundingRect, image) — boundingRect.top() == south — is
    // upright once MapView's negative-Y view flip puts north up on screen. So
    // place south at NDC top (ortho 'top' param = south_y).
    const double west_x = scene_bounds_.left();
    const double east_x = scene_bounds_.right();
    const double south_y = scene_bounds_.top();
    const double north_y = scene_bounds_.bottom();
    QMatrix4x4 mvp;
    mvp.ortho(float(west_x), float(east_x), float(north_y), float(south_y),
              -1.0f, 1.0f);

    program_->bind();
    program_->setUniformValue("u_mvp", mvp);
    program_->setUniformValue("u_radius",
                              float(web_mercator::earth_radius_at_equator));
    program_->setUniformValue("u_min", float(data_min_));
    program_->setUniformValue("u_max", float(data_max_));
    program_->setUniformValue("u_tex", 0);

    const int lonlat_loc = program_->attributeLocation("a_lonlat");
    const int texcoord_loc = program_->attributeLocation("a_texcoord");
    program_->enableAttributeArray(lonlat_loc);
    program_->enableAttributeArray(texcoord_loc);

    // Per-tile triangle strip, subdivided in latitude only. Interleaved
    // [lon, lat, u, v] per vertex.
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
        verts.insert(verts.end(), {float(min_lon), float(lat), 0.0f, v});
        verts.insert(verts.end(), {float(max_lon), float(lat), 1.0f, v});
      }

      QOpenGLTexture* texture = tile->texture();
      if(!texture)
        continue;
      texture->bind(0);
      program_->setAttributeArray(lonlat_loc, GL_FLOAT, verts.data(), 2,
                                  4 * sizeof(float));
      program_->setAttributeArray(texcoord_loc, GL_FLOAT, verts.data() + 2, 2,
                                  4 * sizeof(float));
      f->glDrawArrays(GL_TRIANGLE_STRIP, 0, rows * 2);
      texture->release(0);
    }

    program_->disableAttributeArray(lonlat_loc);
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
    for(auto& tile : tiles_)
      tile->releaseGL();
    gl_context_->doneCurrent();
  }
  delete gl_context_; gl_context_ = nullptr;
  delete gl_surface_; gl_surface_ = nullptr;
}

}  // namespace raster
}  // namespace camp
