#ifndef RASTER_RASTER_LAYER_H
#define RASTER_RASTER_LAYER_H

#include "../map/layer.h"
#include "raster_field_source.h"
#include "raster_gl_renderer.h"

#include <QFutureWatcher>
#include <QImage>
#include <QRectF>
#include <QSize>
#include <memory>
#include <string>
#include <vector>

class QOpenGLTexture;

namespace camp
{
namespace raster
{

/// [#59 ADR-0003 / camp#134] Map layer for a single georeferenced raster chart.
/// GDAL warps the file to Web-Mercator (EPSG:3857) on load; the reprojected pixels
/// are uploaded as a GL texture and drawn through the shared raster::RasterGlRenderer
/// (the same offscreen-FBO path GggsTileLayer and SonarLiveCacheLayer use). Scalar
/// (single-band Float32) charts shade through the colormap LUT with per-cell NoData
/// discard; palette/RGB charts are composited to RGBA8 on the CPU and sampled
/// directly (the LUT is bypassed). Replaces the former QPainter/mipmap path. See
/// ADR-0007.
class RasterLayer: public map::Layer, public RasterFieldSource
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  RasterLayer(map::MapItem* parentItem, const QString& filename);
  ~RasterLayer();

  enum { Type = map::RasterLayerType};
  int type() const override
  {
    return Type;
  }


  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

  /// [camp#63 / camp#141] Select the colour ramp for a scalar (single-band Float32)
  /// raster by marine_colormap palette name; persists and re-renders (just re-bakes
  /// the LUT — no re-warp). No effect on RGB rasters. Unknown name -> grayscale.
  void setColormap(const std::string& name);

  /// [#59 ADR-0003] Source file this layer renders (kept for re-render, removal,
  /// and app-state persistence of the loaded-chart list).
  const QString& filename() const { return filename_; }

  /// [#59 ADR-0003] True once the reprojected extent is known — i.e. the file
  /// opened and warped to EPSG:3857. Valid synchronously after construction
  /// (initExtent is metadata-only), before the async pixel load. False for a
  /// missing or non-georeferenced file.
  bool valid() const { return reprojected_width_ > 0; }

  /// [camp#134] Render the reprojected chart into an offscreen image of @p size
  /// spanning the layer's Web-Mercator extent (boundingRect). Null image if there
  /// is nothing to draw or GL is unavailable. Exposed for headless tests.
  QImage renderImage(const QSize& size);

  /// The layer's Web-Mercator extent. Exposed for tests.
  QRectF sceneBounds() const { return scene_bounds_; }

  // [camp#134] RasterFieldSource: a single reprojected-chart item for the renderer.
  QStringList bands() const override;
  RasterBandMeta metadata(const QString& band) const override;
  QList<RasterFieldItem> items() override;
  QPair<float, float> dataRange() const override;

protected:
  void contextMenu(QMenu* menu) override;
  void readSettings() override;
  void writeSettings() override;
  /// [camp#90] Drop this file from the persisted raster list on user removal.
  void onRemovedFromMap() override;

private:
  // [camp#134] Off-thread load product: the reprojected pixels (scalar R32F values
  // or a composited RGBA QImage) + extent, uploaded to a GL texture on the GUI
  // thread in imageReady().
  struct LoadResult
  {
    bool ok = false;
    bool is_scalar = false;
    int width = 0;
    int height = 0;
    std::vector<float> values;          // scalar path: row-major R32F
    float data_min = 1.0f;              // scalar auto-range (crossed => no data)
    float data_max = 0.0f;
    bool has_nodata = false;
    float nodata = 0.0f;
    QImage rgba;                        // colour path: pre-composited RGBA
    // Web-Mercator extent from the reprojected geotransform (metres).
    double merc_x_min = 0.0, merc_x_max = 0.0;
    double merc_y_min = 0.0, merc_y_max = 0.0;
    int reprojected_width = 0;
    int reprojected_height = 0;
  };

  QFutureWatcher<LoadResult> future_watcher_;

  // Used to indicate the load thread should abort
  bool abort_flag_ = false;
  QMutex abort_flag_mutex_;

  QString filename_;        // kept so a colormap change can re-render
  bool is_scalar_ = false;  // set once loaded; gates the colormap menu

  // [#59 ADR-0003] Reprojected pixel dimensions, set synchronously in the
  // constructor (cheap GDAL metadata) so boundingRect() / fit-to-extent is valid
  // immediately, before the async pixel load finishes. 0 until the extent is known
  // (or on a failed/unreprojectable file).
  int reprojected_width_ = 0;
  int reprojected_height_ = 0;

  // [camp#134] Web-Mercator scene extent (set in initExtent) + the placement
  // convention shared with GggsTileLayer (north-up image anchored at the NW corner
  // with a fromScale(1,-1) item transform). The renderer draws the chart as a
  // single non-geographic quad spanning these metres.
  QRectF scene_bounds_;
  double merc_x_min_ = 0.0, merc_x_max_ = 0.0;
  double merc_y_min_ = 0.0, merc_y_max_ = 0.0;

  // [camp#134] The shared GL renderer (own offscreen context + unified shader +
  // colormap LUT). Default ramp Viridis (scalar charts); set in the ctor.
  RasterGlRenderer renderer_;
  std::unique_ptr<QOpenGLTexture> texture_;   // reprojected chart (R32F or RGBA8)
  RasterFieldItem::Format format_ = RasterFieldItem::Format::Scalar;
  float data_min_ = 1.0f;     // scalar colormap range (crossed => no data)
  float data_max_ = 0.0f;
  bool has_nodata_ = false;
  float nodata_ = 0.0f;

  QImage cached_image_;       // last render, reused on pan (re-rendered on zoom)
  QSize cached_size_;
  static constexpr int kMaxImageEdge = 4096;   // clamp the offscreen target

  // Compute the reprojected extent + Web-Mercator scene placement from file
  // metadata (no pixel reads) and apply them. Synchronous; safe before pixels load.
  void initExtent(const QString& filename);

  LoadResult loadAndReprojectFile(const QString& filename);

private slots:
  void loadFile(const QString& filename);
  void imageReady();

};

} // namespace raster
} // namespace camp

#endif
