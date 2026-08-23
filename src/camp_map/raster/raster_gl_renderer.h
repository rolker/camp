#ifndef RASTER_RASTER_GL_RENDERER_H
#define RASTER_RASTER_GL_RENDERER_H

#include "raster_field_source.h"

#include <QImage>
#include <QList>
#include <QRectF>
#include <QSize>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

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

  /// [camp#141] Select the marine_colormap palette baked into the LUT, by name
  /// (re-baked on next render). An unknown name renders as "grayscale".
  void setColormap(const std::string& name);
  const std::string& colormap() const { return colormap_name_; }

  /// [camp#181 / ADR-0015] Pin the palette's shoreline break to an absolute data
  /// value (chart datum, a tide height, or an operator's manual number), or clear
  /// it. A non-finite anchor is treated as no anchor — storing one would make the
  /// LUT cache key never match again (NaN != NaN), re-baking and re-uploading the
  /// texture every frame for no visible change. Mirrors setColormap's dirty-flag
  /// pattern. The anchor only bites on a palette that declares a
  /// `shoreline_position` (oleron / hypsometric); on any other ramp
  /// `marine_colormap::bake_shoreline_anchored_lut()` falls back to the unanchored
  /// bake, so setting it is harmless. Because the anchored LUT is a function of
  /// the render range,
  /// ensureLut() re-bakes when the anchor OR the [lo, hi] it is baked against
  /// changes — see ensureLut(). Re-baked on next render; shader untouched.
  void setShorelineAnchor(std::optional<float> anchor);
  std::optional<float> shorelineAnchor() const { return shoreline_anchor_; }

  /// [camp#181 / ADR-0015] How many times the LUT has actually been baked (as
  /// opposed to served from the cache). Observability for the cache key, which
  /// ADR-0015 D2 calls out as the silent failure of this design: get it wrong and a
  /// stale LUT renders wrong colours with no crash and no log line, or a
  /// never-matching key re-bakes and re-uploads a texture every frame. Neither is
  /// visible from the outside, so the tests count bakes.
  std::size_t lutBakeCount() const { return lut_bake_count_; }

  /// Release the FBO / program / LUT. Safe to call with no context; the dtor
  /// makes the context current first and then destroys it.
  void releaseGL();

private:
  bool ensureGL();
  bool ensureProgram();
  /// [camp#181 / ADR-0015] Bake (or reuse) the LUT for the render range
  /// [@p lo, @p hi]. The LUT is now range-DEPENDENT when an anchor is active
  /// (ADR-0008 Decision #2, which asserted range-independence, is amended by
  /// ADR-0015): it re-bakes when the palette name, the anchor, OR — while
  /// anchored — [lo, hi] changes. When there is no active anchor the bake ignores
  /// [lo, hi] and reduces to the old range-independent palette ramp, so an
  /// unanchored layer does NOT re-bake on every Auto-range tick.
  QOpenGLTexture* ensureLut(float lo, float hi);

  // Latitude tessellation per geographic item: longitude is linear in
  // Web-Mercator, latitude is the lone nonlinearity. 16 strips is sub-pixel over a
  // tile at these zooms (matches the value previously hard-coded in both layers).
  static constexpr int kLatSubdivisions = 16;

  QOpenGLContext* gl_context_ = nullptr;
  QOffscreenSurface* gl_surface_ = nullptr;
  std::unique_ptr<QOpenGLFramebufferObject> fbo_;
  std::unique_ptr<QOpenGLShaderProgram> program_;
  std::unique_ptr<QOpenGLTexture> lut_texture_;   // colormap LUT (256x1 RGBA)

  std::string colormap_name_{"grayscale"};   // [camp#141] marine_colormap palette
  bool lut_dirty_ = true;                     // name changed (or never baked)
  // [camp#181 / ADR-0015] The active shoreline anchor and the cache key of the
  // LUT currently uploaded. lut_anchor_/lut_lo_/lut_hi_ record what the texture
  // was baked against so ensureLut() can detect an anchor or range change; they
  // are meaningful only while lut_dirty_ is false.
  std::optional<float> shoreline_anchor_;
  std::optional<float> lut_anchor_;
  float lut_lo_ = 0.0f;
  float lut_hi_ = 0.0f;
  std::size_t lut_bake_count_ = 0;            // see lutBakeCount()
  bool gl_failed_ = false;
};

}  // namespace raster
}  // namespace camp

#endif
