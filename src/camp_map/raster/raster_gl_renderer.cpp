#include "raster_gl_renderer.h"

#include "../map_view/web_mercator.h"

#include <QColor>
#include <QGeoCoordinate>
#include <QMatrix4x4>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

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

// [camp#134] The ONE unified fragment shader (consolidated from GggsTileLayer +
// SonarLiveCacheLayer, and now serving RasterLayer too). Two modes:
//
//  - Scalar (u_mode == 0): single-band value texture (R32F) auto-ranged and mapped
//    through the colormap LUT. The NaN NoData fix lands HERE, once: `v != v`
//    discards NaN sentinels (chart/backscatter stores) — the GLSL 1.20-portable
//    idiom (isnan() is 1.30+). The finite-sentinel discard (v == u_nodata, e.g.
//    sidescan 9999/0) follows. Premultiplied-opaque output (alpha 1).
//
//  - Rgba (u_mode == 1): a pre-composited, PREMULTIPLIED RGBA8 texture sampled
//    directly, BYPASSING the LUT (RasterLayer's palette/RGB charts). The uploader
//    premultiplies (so mipmap/linear filtering doesn't fringe), so the texel is
//    emitted as-is — already matching the GL_ONE / GL_ONE_MINUS_SRC_ALPHA blend.
//    Transparency comes from the composited alpha; fully-transparent cells discard.
constexpr char kFragmentShader[] = R"(
#version 120
uniform sampler2D u_tex;     // unit 0: Scalar R32F data, or Rgba RGBA8
uniform sampler2D u_lut;     // unit 1: colormap LUT (256x1 RGBA)
uniform float u_min;
uniform float u_max;
uniform int u_has_nodata;    // Scalar: nonzero => discard v == u_nodata
uniform float u_nodata;
uniform int u_mode;          // 0 = Scalar (LUT), 1 = Rgba (direct)
varying vec2 v_texcoord;
void main()
{
  if(u_mode == 0)
  {
    float v = texture2D(u_tex, v_texcoord).r;
    if(v != v)
      discard;                                       // NaN NoData (1.20-portable)
    if(u_has_nodata != 0 && v == u_nodata)
      discard;                                        // finite sentinel
    // Normalize over the TRUE data span (matches ColorMap::color), so sub-unit
    // ranges still stretch across the colormap. The 1e-6 floor is ONLY a
    // divide-by-zero guard for a genuinely degenerate (zero-width) range — a true
    // span of 0 collapses to t=0 (a flat LUT value). The old 1.0 floor silently
    // crushed contrast for any span < 1.0 (harmless for large-range GGGS/sidescan
    // depth, but wrong once small-range RasterLayer charts share this shader).
    // Negative-valued ranges are fine: span = u_max - u_min stays positive.
    float t = clamp((v - u_min) / max(u_max - u_min, 1e-6), 0.0, 1.0);
    vec4 c = texture2D(u_lut, vec2(t, 0.5));
    gl_FragColor = vec4(c.rgb, 1.0);
  }
  else
  {
    vec4 c = texture2D(u_tex, v_texcoord);   // already premultiplied
    if(c.a == 0.0)
      discard;
    gl_FragColor = c;
  }
}
)";

}  // namespace

RasterGlRenderer::RasterGlRenderer() = default;

RasterGlRenderer::~RasterGlRenderer()
{
  if(gl_context_ && gl_surface_ && gl_context_->makeCurrent(gl_surface_))
  {
    releaseGL();
    gl_context_->doneCurrent();
  }
  delete gl_context_; gl_context_ = nullptr;
  delete gl_surface_; gl_surface_ = nullptr;
}

bool RasterGlRenderer::ensureGL()
{
  // gl_failed_ first: once GL is declared broken (creation OR a mid-session
  // makeCurrent failure), stay failed and don't retry — otherwise every repaint
  // re-enters and re-warns.
  if(gl_failed_)
    return false;
  if(gl_context_)
    return true;

  gl_surface_ = new QOffscreenSurface();
  gl_surface_->create();
  gl_context_ = new QOpenGLContext();
  if(!gl_surface_->isValid() || !gl_context_->create())
  {
    qWarning("RasterGlRenderer: offscreen GL unavailable; raster not rendered");
    gl_failed_ = true;
    delete gl_context_; gl_context_ = nullptr;
    delete gl_surface_; gl_surface_ = nullptr;
    return false;
  }
  return true;
}

bool RasterGlRenderer::makeCurrent()
{
  if(!ensureGL())
    return false;
  if(!gl_context_->makeCurrent(gl_surface_))
  {
    qWarning("RasterGlRenderer: makeCurrent failed; raster not rendered");
    gl_failed_ = true;
    return false;
  }
  return true;
}

void RasterGlRenderer::doneCurrent()
{
  if(gl_context_)
    gl_context_->doneCurrent();
}

bool RasterGlRenderer::ensureProgram()
{
  if(program_)
    return program_->isLinked();
  program_ = std::make_unique<QOpenGLShaderProgram>();
  program_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader);
  program_->addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader);
  if(!program_->link())
  {
    qWarning("RasterGlRenderer: shader link failed: %s",
             program_->log().toUtf8().constData());
    return false;
  }
  return true;
}

QOpenGLTexture* RasterGlRenderer::ensureLut()
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

void RasterGlRenderer::setColormap(map::ColorMap::Type type)
{
  if(type == colormap_.type())
    return;
  colormap_.setType(type);
  lut_dirty_ = true;
}

QImage RasterGlRenderer::renderToImage(const QList<RasterFieldItem>& items,
                                       const QRectF& scene_bounds, float data_min,
                                       float data_max, const QSize& size)
{
  if(items.isEmpty() || size.isEmpty() || !gl_context_)
    return QImage();

  // [camp#134] Fail BEFORE allocating/binding the FBO so a shader-compile failure
  // returns a null image (per the header contract) — not a transparent one the
  // caller would cache as a valid empty render.
  if(!ensureProgram())
    return QImage();

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

  {
    // Vertices are LOCAL Web-Mercator metres from the extent origin, so map
    // [0, width] x [0, height] -> NDC. origin = scene_bounds top-left; the per-item
    // warp subtracts it so the float positions stay small (precise at large
    // Web-Mercator magnitudes). The layer's fromScale(1,-1) + NW anchor draws the
    // result upright (north-up).
    const double origin_x = scene_bounds.left();
    const double origin_y = scene_bounds.top();
    const double width_m = scene_bounds.width();
    const double height_m = scene_bounds.height();
    QMatrix4x4 mvp;
    mvp.ortho(0.0f, float(width_m), 0.0f, float(height_m), -1.0f, 1.0f);

    program_->bind();
    program_->setUniformValue("u_mvp", mvp);
    program_->setUniformValue("u_min", data_min);
    program_->setUniformValue("u_max", data_max);
    program_->setUniformValue("u_tex", 0);
    program_->setUniformValue("u_lut", 1);
    QOpenGLTexture* lut = ensureLut();
    if(lut)
      lut->bind(1);

    const int pos_loc = program_->attributeLocation("a_pos");
    const int texcoord_loc = program_->attributeLocation("a_texcoord");
    program_->enableAttributeArray(pos_loc);
    program_->enableAttributeArray(texcoord_loc);

    std::vector<float> verts;
    for(const RasterFieldItem& item : items)
    {
      if(!item.texture)
        continue;

      // Per-item triangle strip. A geographic item subdivides in latitude and warps
      // each row to Web-Mercator (the GGGS / live path); a non-geographic item is
      // already in Web-Mercator metres, so a single linear quad (2 rows) suffices
      // (the RasterLayer path, reprojected by GDAL on load). Interleaved
      // [local_x, local_y, u, v] per vertex; texture row 0 = north (v = 0).
      const int rows = item.geographic ? (kLatSubdivisions + 1) : 2;
      verts.clear();
      verts.reserve(rows * 2 * 4);
      for(int r = 0; r < rows; ++r)
      {
        const double frac = double(r) / (rows - 1);
        const float v = float(frac);                   // tex row 0 = north
        QPointF l, rt;
        if(item.geographic)
        {
          const double lat = item.north + (item.south - item.north) * frac;
          l = web_mercator::geoToMap(QGeoCoordinate(lat, item.west));
          rt = web_mercator::geoToMap(QGeoCoordinate(lat, item.east));
        }
        else
        {
          // Already Web-Mercator metres; north (max y) at frac 0, linear to south.
          const double y = item.north + (item.south - item.north) * frac;
          l = QPointF(item.west, y);
          rt = QPointF(item.east, y);
        }
        verts.insert(verts.end(),
                     {float(l.x() - origin_x), float(l.y() - origin_y), 0.0f, v});
        verts.insert(verts.end(),
                     {float(rt.x() - origin_x), float(rt.y() - origin_y), 1.0f, v});
      }

      const bool scalar = (item.format == RasterFieldItem::Format::Scalar);
      // [camp#134] Data-texture filter set HERE, once. Scalar: Nearest so the
      // exact-equality NoData discard never blends across a sentinel boundary (the
      // halo bug, camp#122). Rgba: Linear, and LinearMipMapLinear when the source
      // supplied mipmaps (RasterLayer's charts) — that restores the LOD the old
      // QPainter mipmap pyramid provided, so a chart zoomed far out doesn't shimmer.
      if(scalar)
        item.texture->setMinMagFilters(QOpenGLTexture::Nearest,
                                       QOpenGLTexture::Nearest);
      else
        item.texture->setMinMagFilters(
          item.texture->mipLevels() > 1 ? QOpenGLTexture::LinearMipMapLinear
                                        : QOpenGLTexture::Linear,
          QOpenGLTexture::Linear);
      item.texture->bind(0);
      program_->setUniformValue("u_mode", scalar ? 0 : 1);
      program_->setUniformValue("u_has_nodata",
                                (scalar && item.has_nodata) ? 1 : 0);
      program_->setUniformValue("u_nodata",
                                (scalar && item.has_nodata) ? item.nodata : 0.0f);
      program_->setAttributeArray(pos_loc, GL_FLOAT, verts.data(), 2,
                                  4 * sizeof(float));
      program_->setAttributeArray(texcoord_loc, GL_FLOAT, verts.data() + 2, 2,
                                  4 * sizeof(float));
      f->glDrawArrays(GL_TRIANGLE_STRIP, 0, rows * 2);
      item.texture->release(0);
    }

    if(lut)
      lut->release(1);
    program_->disableAttributeArray(pos_loc);
    program_->disableAttributeArray(texcoord_loc);
    program_->release();
  }

  fbo_->release();
  QImage image = fbo_->toImage();   // top-down ARGB32 (premultiplied)
  return image;
}

void RasterGlRenderer::releaseGL()
{
  fbo_.reset();
  program_.reset();
  lut_texture_.reset();
}

}  // namespace raster
}  // namespace camp
