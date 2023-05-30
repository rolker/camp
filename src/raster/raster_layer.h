#ifndef RASTER_RASTER_LAYER_H
#define RASTER_RASTER_LAYER_H

#include "../map/layer.h"
#include <QFutureWatcher>

namespace raster
{

class RasterLayer: public map::Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  RasterLayer(map::MapItem* parentItem);

  enum { Type = map::RasterLayerType};
  int type() const override
  {
    return Type;
  }

  
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

public slots:
  void loadFile(const QString& filename);

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
  };

  QFutureWatcher<LoadResult> future_watcher_;

  LoadResult loadAndReprojectFile(const QString& filename) const;

private slots:
  void imageReady();

};

} // namepsace raster

#endif
