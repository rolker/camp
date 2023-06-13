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
  RasterLayer(map::MapItem* parentItem, const QString& filename);
  ~RasterLayer();

  enum { Type = map::RasterLayerType};
  int type() const override
  {
    return Type;
  }

  
  QRectF boundingRect() const override;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;


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

  // Used to indicate the load thread should abort
  bool abort_flag_ = false;
  QMutex abort_flag_mutex_;

  LoadResult loadAndReprojectFile(const QString& filename);

private slots:
  void loadFile(const QString& filename);
  void imageReady();

};

} // namepsace raster

#endif
