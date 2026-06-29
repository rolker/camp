#ifndef RASTER_RASTER_GL_RENDERER_H
#define RASTER_RASTER_GL_RENDERER_H

#include "raster_field_source.h"
#include "../map/color_map.h"

#include <QImage>
#include <QList>
#include <QRectF>
#include <QSize>
#include <memory>

class QOpenGLContext;
class QOffscreenSurface;
class QOpenGLFramebufferObject;
class QOpenGLShaderProgram;
class QOpenGLTexture;

namespace camp
{
namespace raster
{

/// [camp#134] The ONE shared GL raster render path (see ADR-0007). Owns the
/// per-GL-context state — an offscreen context + surface, the FBO, the single
/// compiled shader program, and the colormap LUT texture — and the unified
/// fragment shader. The geo→Web-Mercator vertex tessellation and the data-texture
/// filter (Nearest for Scalar, Linear for Rgba) live here so GggsTileLayer,
/// SonarLiveCacheLayer and RasterLayer no longer each carry a copy.
///
/// Each layer holds one RasterGlRenderer (so each layer keeps its own offscreen
/// context, as before). Usage from a layer's renderImage():
/// ```
///   if(!renderer_.makeCurrent()) return QImage();   // GL unavailable -> null
///   QList<RasterFieldItem> items = collectItems();  // uploads textures here
///   QImage img = renderer_.renderToImage(items, scene_bounds_, min, max, size);
///   renderer_.doneCurrent();
/// ```
/// The textures in @p items are owned by the caller's sources (GggsTile, live
/// Entry, RasterLayer); the renderer never takes ownership.
class RasterGlRenderer
{
public:
  RasterGlRenderer();
  ~RasterGlRenderer();

  RasterGlRenderer(const RasterGlRenderer&) = delete;
  RasterGlRenderer& operator=(const RasterGlRenderer&) = delete;

  /// Lazily create + make current this renderer's offscreen GL context. Returns
  /// false (latched, so it never re-warns) if GL is unavailable or makeCurrent
  /// fails. A caller releases its own source textures between makeCurrent() and
  /// doneCurrent() (teardown / band switch); items() likewise uploads under it.
  bool makeCurrent();
  void doneCurrent();

  /// Whether the offscreen context already exists (created on first makeCurrent()).
  /// Lets a caller skip a texture-release pass before the layer has ever rendered
  /// (no context, no textures) without forcing the context into existence early.
  bool hasContext() const { return gl_context_ != nullptr; }

  /// Warp @p items into an offscreen image of @p size spanning @p scene_bounds
  /// (the layer's Web-Mercator extent), with the colormap range [@p data_min,
  /// @p data_max] for Scalar items. The context MUST already be current (see
  /// makeCurrent()). Returns a null image if the shader fails to compile.
  QImage renderToImage(const QList<RasterFieldItem>& items,
                       const QRectF& scene_bounds, float data_min, float data_max,
                       const QSize& size);

  /// Select the colour ramp baked into the LUT (re-baked on next render).
  void setColormap(map::ColorMap::Type type);
  map::ColorMap::Type colormap() const { return colormap_.type(); }

  /// Release the FBO / program / LUT. Safe to call with no context; the dtor
  /// makes the context current first and then destroys it.
  void releaseGL();

private:
  bool ensureGL();
  bool ensureProgram();
  QOpenGLTexture* ensureLut();

  // Latitude tessellation per geographic item: longitude is linear in
  // Web-Mercator, latitude is the lone nonlinearity. 16 strips is sub-pixel over a
  // tile at these zooms (matches the value previously hard-coded in both layers).
  static constexpr int kLatSubdivisions = 16;

  QOpenGLContext* gl_context_ = nullptr;
  QOffscreenSurface* gl_surface_ = nullptr;
  std::unique_ptr<QOpenGLFramebufferObject> fbo_;
  std::unique_ptr<QOpenGLShaderProgram> program_;
  std::unique_ptr<QOpenGLTexture> lut_texture_;   // colormap LUT (256x1 RGBA)

  map::ColorMap colormap_{map::ColorMap::Grayscale};
  bool lut_dirty_ = true;
  bool gl_failed_ = false;
};

}  // namespace raster
}  // namespace camp

#endif
