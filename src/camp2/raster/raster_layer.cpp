#include "raster_layer.h"
#include <gdal_priv.h>
#include <gdalwarper.h>
#include "../map_view/web_mercator.h"
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QtConcurrent>

#include <QDebug>

namespace raster
{

RasterLayer::RasterLayer(map::MapItem* parentItem, const QString& filename):
  map::Layer(parentItem, QFileInfo(filename).fileName())
{
  if(GDALGetDriverCount() == 0)
    GDALAllRegister();
  connect(&future_watcher_, &QFutureWatcher<LoadResult>::finished, this, &RasterLayer::imageReady);
  loadFile(filename);
}

RasterLayer::~RasterLayer()
{
  abort_flag_mutex_.lock();
  abort_flag_ = true;
  abort_flag_mutex_.unlock();
  future_watcher_.waitForFinished();
}

QRectF RasterLayer::boundingRect() const
{
  if(!mipmaps_.empty())
    return mipmaps_.begin()->second.rect();
  return QRectF();
}


void RasterLayer::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  if(!mipmaps_.empty())
  {
    auto lod = QStyleOptionGraphicsItem::levelOfDetailFromTransform(painter->worldTransform());
    auto level = mipmaps_.lower_bound( std::min(255, int(1.0/lod)) );
    if(level == mipmaps_.end())
      level--;
    painter->save();
    painter->setRenderHint(QPainter::SmoothPixmapTransform);
    painter->scale(level->first, level->first);
    painter->drawPixmap(0,0, level->second);
    painter->restore();
  }

}


void RasterLayer::loadFile(const QString& filename)
{
  setStatus("(loading...)");
  future_watcher_.setFuture(QtConcurrent::run(this, &RasterLayer::loadAndReprojectFile, filename));
}

RasterLayer::LoadResult RasterLayer::loadAndReprojectFile(const QString& filename)
{
  LoadResult result;

  auto dataset = GDALDataset::FromHandle(GDALOpen(filename.toLatin1(), GA_ReadOnly));

  if(!dataset)
  {
    qDebug("RasterLayer::loadFile null dataset");
    return result;
  }

  auto reprojected_dataset = GDALDataset::FromHandle(GDALAutoCreateWarpedVRT(dataset, nullptr, web_mercator::wkt, GRA_Bilinear, 0.0, nullptr));

  if(!reprojected_dataset)
  {
    qDebug("RasterLayer::loadFile error creating repojected dataset");
    return result;
  }

  double reprojected_geo_transform[6] = {0.0};
  reprojected_dataset->GetGeoTransform(reprojected_geo_transform);

  result.world_x = reprojected_geo_transform[0];
  result.world_y = reprojected_geo_transform[3];

  result.scale_x = reprojected_geo_transform[1];
  result.scale_y = reprojected_geo_transform[5];

  auto width = reprojected_dataset->GetRasterXSize();
  auto height = reprojected_dataset->GetRasterYSize();

  QImage image(width, height, QImage::Format_ARGB32);
  image.fill(Qt::black);

  for(auto&& band: reprojected_dataset->GetBands())
  {
    auto color_table = band->GetColorTable();
    std::vector<uint32_t> buffer(width);
    for(int j = 0; j<height; ++j)
    {
      if(band->RasterIO(GF_Read, 0, j, width, 1, &buffer.front(), width, 1, GDT_UInt32, 0, 0)== CE_None)
      {
        uchar *scanline = image.scanLine(j);
        for(int i = 0; i < width; ++i)
        {
          if(color_table)
          {
            GDALColorEntry const *ce = color_table->GetColorEntry(buffer[i]);
            scanline[i*4] = ce->c3;
            scanline[i*4+1] = ce->c2;
            scanline[i*4+2] = ce->c1;
            scanline[i*4+3] = ce->c4;
          }
          else
          {
            if(band->GetColorInterpretation() == GCI_GrayIndex)
            {
                scanline[i*4+0] = buffer[i];
                scanline[i*4+1] = buffer[i];
                scanline[i*4+2] = buffer[i];
            }
            if(band->GetColorInterpretation() == GCI_RedBand)
                scanline[i*4+2] = buffer[i];
            if(band->GetColorInterpretation() == GCI_GreenBand)
                scanline[i*4+1] = buffer[i];
            if(band->GetColorInterpretation() == GCI_BlueBand)
                scanline[i*4+0] = buffer[i];
            if(band->GetColorInterpretation() == GCI_AlphaBand)
                scanline[i*4+3] = buffer[i];
          }
        }
      }
      QMutexLocker lock(&abort_flag_mutex_);
      if(abort_flag_)
        return {};
    }
  }

  result.mipmaps[1] = QPixmap::fromImage(image);
  for(int i = 2; i < 128; i*=2)
  {
    result.mipmaps[i] = QPixmap::fromImage(image.scaledToWidth(width/float(i),Qt::SmoothTransformation));
  }
  return result;
}

void RasterLayer::imageReady()
{
  auto result = future_watcher_.result();
  prepareGeometryChange();

  mipmaps_ = result.mipmaps;

  setTransform(QTransform::fromScale(result.scale_x, result.scale_y), true);
  setPos(result.world_x, result.world_y);
 
  update(boundingRect());
  setStatus("");
}

} // namespace raster
