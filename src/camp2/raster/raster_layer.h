#ifndef RASTER_RASTER_LAYER_H
#define RASTER_RASTER_LAYER_H

#include "../map/layer.h"
#include "../map/color_map.h"
#include <QFutureWatcher>

namespace camp
{
namespace raster
{

class RasterLayer: public map::Layer
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

  /// [camp#63] Select the colour ramp for a scalar (single-band Float32) raster;
  /// persists and re-renders. No effect on RGB rasters.
  void setColormap(map::ColorMap::Type type);

  /// [#59 ADR-0003] Source file this layer renders (kept for re-render, removal,
  /// and app-state persistence of the loaded-chart list).
  const QString& filename() const { return filename_; }

  /// [#59 ADR-0003] True once the reprojected extent is known — i.e. the file
  /// opened and warped to EPSG:3857. Valid synchronously after construction
  /// (initExtent is metadata-only), before the async pixel load. False for a
  /// missing or non-georeferenced file.
  bool valid() const { return reprojected_width_ > 0; }

protected:
  void contextMenu(QMenu* menu) override;
  void readSettings() override;
  void writeSettings() override;

private:
  // Store lower resolutions in an image pyramid
  using Mipmaps = std::map<uint8_t, QPixmap>;
  Mipmaps mipmaps_;

  struct LoadResult
  {
    Mipmaps mipmaps;
    double world_x;
    double world_y;
    double scale_x;
    double scale_y;
    bool is_scalar = false;   // single-band Float32 (depth) -> ColorMap-shaded
  };

  QFutureWatcher<LoadResult> future_watcher_;

  // Used to indicate the load thread should abort
  bool abort_flag_ = false;
  QMutex abort_flag_mutex_;

  // [camp#63/#59 PR3c] Colour ramp for single-band scalar (e.g. depth) rasters,
  // which would otherwise render as near-black via the UInt32 RGB path.
  map::ColorMap colormap_{map::ColorMap::Viridis};
  QString filename_;        // kept so a colormap change can re-render
  bool is_scalar_ = false;  // set once loaded; gates the colormap menu

  // [#59 ADR-0003] Reprojected pixel dimensions, set synchronously in the
  // constructor (cheap GDAL metadata) so boundingRect() — and therefore
  // sceneBoundingRect() / fit-to-extent — is valid immediately, before the
  // async pixel/mipmap build finishes. 0 until the extent is known (or on a
  // failed/unreprojectable file).
  int reprojected_width_ = 0;
  int reprojected_height_ = 0;

  // Compute the reprojected extent + world transform/pos from file metadata
  // (no pixel reads) and apply them. Synchronous; safe before pixels load.
  void initExtent(const QString& filename);

  LoadResult loadAndReprojectFile(const QString& filename);

private slots:
  void loadFile(const QString& filename);
  void imageReady();

};

} // namespace raster
} // namespace camp

#endif
