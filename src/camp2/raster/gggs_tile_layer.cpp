#include "gggs_tile_layer.h"

#include "gggs_tile.h"
#include "../map_view/web_mercator.h"

#include <QDir>
#include <QGeoCoordinate>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLWidget>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QFileInfo>

#include <algorithm>
#include <array>
#include <cmath>

namespace camp
{
namespace raster
{

namespace
{

// Vertex shader: per-vertex geo→Web-Mercator warp. This is a line-by-line port
// of web_mercator::geoToMap (x = R·λ; y = R·asinh(tan φ)); asinh is expanded as
// log(t + sqrt(t²+1)) to stay valid on GLSL profiles without the asinh builtin.
// The geoToMap-parity unit test pins the formula so the shader can't silently
// drift from the C++ reference.
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
// reserved by the mosaicker; real returns are floored to >= 1) is discarded so
// empty cells are transparent. Premultiplied-alpha output to composite under
// Qt's GL paint engine. Band-select + real colormap are Slice 3 (camp#63).
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
  if(tiles_.empty())
    setStatus("(no tiles)");
}

GggsTileLayer::~GggsTileLayer()
{
  releaseGL();
}

void GggsTileLayer::releaseGL()
{
  // GL objects must be freed with their context current. If the viewport widget
  // is still alive, make its context current; otherwise the objects fall to
  // their destructors (a QOpenGLTexture with no current context warns + leaks,
  // which only happens at app teardown when the context is gone anyway).
  if(!program_ && tiles_.empty())
    return;
  const bool have_ctx = gl_widget_ && gl_widget_->context();
  if(have_ctx)
    gl_widget_->makeCurrent();
  program_.reset();
  for(auto& tile : tiles_)
    tile->releaseGL();
  if(have_ctx)
    gl_widget_->doneCurrent();
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
  return scene_bounds_;
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
    setStatus("(shader error)");
    return false;
  }
  return true;
}

void GggsTileLayer::paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget* widget)
{
  if(tiles_.empty())
    return;
  // Auto-range needs valid samples; a fully-NoData mosaic has nothing to draw.
  if(data_min_ > data_max_)
    return;

  painter->beginNativePainting();

  if(!gl_ready_)
  {
    initializeOpenGLFunctions();
    gl_ready_ = true;
  }
  if(!gl_widget_)
    gl_widget_ = qobject_cast<QOpenGLWidget*>(widget);

  if(ensureProgram())
  {
    // MVP: scene(Web-Mercator) -> viewport logical px (the live QPainter world
    // transform, which carries MapView's negative-Y flip + pan/zoom) -> NDC. The
    // GL viewport covers the full framebuffer, so normalizing by logical size is
    // device-pixel-ratio independent.
    const QTransform w = painter->worldTransform();
    const QMatrix4x4 world(
      w.m11(), w.m21(), 0.0f, w.dx(),
      w.m12(), w.m22(), 0.0f, w.dy(),
      0.0f, 0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 0.0f, 1.0f);
    const double vw = widget ? widget->width() : painter->device()->width();
    const double vh = widget ? widget->height() : painter->device()->height();
    QMatrix4x4 ortho;
    ortho.ortho(0.0f, float(vw), float(vh), 0.0f, -1.0f, 1.0f);
    const QMatrix4x4 mvp = ortho * world;

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

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

    // Per-tile triangle strip, subdivided in latitude only (longitude is the
    // linear axis of the warp). Interleaved [lon, lat, u, v] per vertex.
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
        // left edge (u = 0), then right edge (u = 1)
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
      glDrawArrays(GL_TRIANGLE_STRIP, 0, rows * 2);
      texture->release(0);
    }

    program_->disableAttributeArray(lonlat_loc);
    program_->disableAttributeArray(texcoord_loc);
    program_->release();
  }

  painter->endNativePainting();
}

}  // namespace raster
}  // namespace camp
