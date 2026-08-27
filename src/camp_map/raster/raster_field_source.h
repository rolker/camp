#ifndef RASTER_RASTER_FIELD_SOURCE_H
#define RASTER_RASTER_FIELD_SOURCE_H

#include <QList>
#include <QPair>
#include <QString>
#include <QStringList>

class QOpenGLTexture;

namespace camp
{
namespace raster
{

/// [camp#134] One renderable raster patch handed to RasterGlRenderer. See ADR-0007.
///
/// The texture is OWNED BY THE SOURCE (a GggsTile, a live-cache Entry, or a
/// RasterLayer); the pointer is non-owning and valid only for the duration of the
/// renderToImage() call it is passed to.
///
/// A `Scalar` item carries a single-band R32F value texture shaded through the
/// colormap LUT (with NaN + finite-sentinel NoData discard). An `Rgba` item
/// carries a pre-composited RGBA8 texture sampled directly — the LUT is bypassed
/// and transparency comes from the composited alpha (per-band channel select for
/// colour files is a future issue; RasterLayer composites on the CPU as today).
struct RasterFieldItem
{
  enum class Format
  {
    Scalar,   ///< R32F value texture -> colormap LUT
    Rgba      ///< pre-composited RGBA8 texture, LUT bypassed
  };

  QOpenGLTexture* texture = nullptr;
  Format format = Format::Scalar;

  /// Extent of the patch. When `geographic` is true the bounds are degrees and the
  /// renderer subdivides in latitude and warps each vertex to Web-Mercator
  /// (web_mercator::geoToMap) — the GGGS / live-tile path, where tiles stay in
  /// lat/lon and are warped at display time. When false the bounds are already
  /// Web-Mercator metres (a single linear quad, no warp) — the RasterLayer path,
  /// where GDAL reprojected the chart to EPSG:3857 on load.
  bool geographic = true;
  double west = 0.0;    ///< min lon (deg) or min x (metres)
  double east = 0.0;    ///< max lon (deg) or max x (metres)
  double south = 0.0;   ///< min lat (deg) or min y (metres)
  double north = 0.0;   ///< max lat (deg) or max y (metres)

  /// Scalar only: discard cells exactly equal to `nodata` (the finite sentinel).
  /// NaN cells are always discarded by the shader regardless of these.
  bool has_nodata = false;
  float nodata = 0.0f;

  /// [field 2026-08-27] Sub-rect of the texture to stretch across the bounds above,
  /// in normalized texture coordinates: `u` runs west→east, `v` runs north→south
  /// (texture row 0 = north, matching the renderer's vertex generation). The
  /// default (0,0)-(1,1) is the whole texture — exactly what every item did before
  /// this field existed, so `RasterLayer` and `GggsTileLayer` are unaffected.
  ///
  /// It exists for the live-coverage **coarse placeholder**: an overview tile may
  /// only ever be painted over the footprint of ONE not-yet-loaded fine tile
  /// (ADR-0010 D5), which is a sub-rectangle of the overview, not the whole of it.
  /// Narrowing the bounds alone cannot express that — the renderer stretches the
  /// full texture across whatever bounds it is given, so the coarse tile would be
  /// squashed into the small box instead of clipped to it.
  float u0 = 0.0f;   ///< west edge of the sampled sub-rect
  float v0 = 0.0f;   ///< north edge of the sampled sub-rect
  float u1 = 1.0f;   ///< east edge of the sampled sub-rect
  float v1 = 1.0f;   ///< south edge of the sampled sub-rect
};

/// [camp#134] Per-band metadata, surfaced for the (per-layer) band picker.
struct RasterBandMeta
{
  bool has_nodata = false;
  float nodata = 0.0f;
  QString units;
};

/// [camp#134] Source-agnostic supplier of renderable raster items for
/// RasterGlRenderer. GggsTileLayer (GDAL tiles), SonarLiveCacheLayer (in-memory
/// live tiles) and RasterLayer (a single reprojected chart) implement it so the GL
/// shader, the geo→Web-Mercator tessellation, and the colormap LUT live ONCE in
/// the renderer rather than being duplicated per layer. See ADR-0007.
///
/// **Threading / ownership.** `items()` is called on the GUI/GL thread with the
/// renderer's context current, so a source may lazily upload its textures inside
/// it; the returned texture pointers are owned by the source and stay valid until
/// the enclosing renderToImage() call completes.
class RasterFieldSource
{
public:
  virtual ~RasterFieldSource() = default;

  /// Selectable bands (e.g. {"1","2"} for GggsTile's 1-indexed bands, or the
  /// named live-cache bands). Backs the per-layer band picker.
  virtual QStringList bands() const = 0;

  /// NoData sentinel + units for @p band (one of bands()).
  virtual RasterBandMeta metadata(const QString& band) const = 0;

  /// Items to draw for the current selection. Textures are uploaded lazily here
  /// (context current) and owned by the source.
  virtual QList<RasterFieldItem> items() = 0;

  /// Colormap range [min,max] over the Scalar items; crossed (min > max) when
  /// there is no data yet. Ignored by Rgba items.
  virtual QPair<float, float> dataRange() const = 0;
};

}  // namespace raster
}  // namespace camp

#endif
