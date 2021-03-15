#include "backgroundraster.h"
#include <QPainter>
#include <QJsonObject>
#include <gdal_priv.h>
#include <QModelIndex>
#include <QDebug>

BackgroundRaster::BackgroundRaster(const QString &fname, QObject *parent, QGraphicsItem *parentItem)
    : MissionItem(parent), QGraphicsItem(parentItem), m_filename(fname),m_valid(false),m_width(0),m_height(0)
{
    GDALDataset * dataset = reinterpret_cast<GDALDataset*>(GDALOpen(fname.toStdString().c_str(),GA_ReadOnly));
    if (dataset)
    {
        extractGeoreference(dataset);

        m_width = dataset->GetRasterXSize();
        m_height = dataset->GetRasterYSize();
        
        QGeoCoordinate p1 = pixelToGeo(QPointF(m_width/2,m_height/2));
        QGeoCoordinate p2 = pixelToGeo(QPointF((m_width/2)+1,m_height/2));
        m_pixel_size = p1.distanceTo(p2);
        qDebug() << "pixel size: " << m_pixel_size;
        
        QImage image(m_width,m_height,QImage::Format_ARGB32);
        image.fill(Qt::black);
        
        for(int bandNumber = 1; bandNumber <= dataset->GetRasterCount(); bandNumber++)
        {
            GDALRasterBand * band = dataset->GetRasterBand(bandNumber);

            // is it a depth layer?
            if(band->GetRasterDataType() == GDT_Float32 && !depthValid())
            {
                m_depth_data.resize(m_width*m_height);
                if(band->RasterIO(GF_Read,0,0,m_width,m_height,&m_depth_data.front(),m_width,m_height,GDT_Float32,0,0) != CE_None)
                    m_depth_data.clear();
                else
                {
                    double minmax[2];
                    if(band->ComputeRasterMinMax(false,minmax) == CE_None)
                    {
                        qDebug() << "Depth layer: min: " << minmax[0] << " max: " << minmax[1];
                        for(int j = 0; j<m_height; ++j)
                        {
                            uchar *scanline = image.scanLine(j);
                            for(int i = 0; i < m_width; ++i)
                            {
                                float depth = m_depth_data[j*m_width+i];
                                if (depth <= 0.0)
                                {
                                    scanline[i*4] = 64;
                                    scanline[i*4+1] = 100;
                                    scanline[i*4+2] = 2;
                                    scanline[i*4+3] = 255;
                                }
                                else
                                {
                                    scanline[i*4] = 255;
                                    scanline[i*4+1] = 255*(1-(depth/minmax[1]));
                                    scanline[i*4+2] = 255*(1-(depth/minmax[1]));
                                    scanline[i*4+3] = 255;
                                }
                            }
                        }
                    }
                    
                }
            }
            else
            {
                

                GDALColorTable *colorTable = band->GetColorTable();

                std::vector<uint32_t> buffer(m_width);


                for(int j = 0; j<m_height; ++j)
                {
                    if(band->RasterIO(GF_Read,0,j,m_width,1,&buffer.front(),m_width,1,GDT_UInt32,0,0)==CE_None)
                    {
                        uchar *scanline = image.scanLine(j);
                        for(int i = 0; i < m_width; ++i)
                        {
                            if(colorTable)
                            {
                                GDALColorEntry const *ce = colorTable->GetColorEntry(buffer[i]);
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

                                //scanline[i*4+3] = 255; // hack to make sure alpha is solid in case no alpha channel is present
                                //scanline[i*4 + 2-(bandNumber-1)] = buffer[i];
                            }
                        }
                    }
                }
            }
        }
        
        backgroundImages[1] = QPixmap::fromImage(image);
        for(int i = 2; i < 128; i*=2)
        {
            backgroundImages[i] = QPixmap::fromImage(image.scaledToWidth(m_width/float(i),Qt::SmoothTransformation));
        }
        m_valid = true;
    }
}

bool BackgroundRaster::valid() const
{
    return m_valid;
}

bool BackgroundRaster::depthValid() const
{
    return m_width > 0 && m_height > 0 && m_depth_data.size() == m_width*m_height;
}


QRectF BackgroundRaster::boundingRect() const
{
    auto ret = backgroundImages.cbegin();
    return QRectF(QPointF(0.0,0.0), ret->second.size());
}


void BackgroundRaster::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,QWidget *widget)
{
    painter->save();
    painter->setRenderHint(QPainter::SmoothPixmapTransform);
    double scale = painter->transform().m11();
    if(!backgroundImages.empty())
    {
        QPixmap selectedBackground;
        int selectedScale;
        auto l = backgroundImages.lower_bound(1/scale);
        if (l == backgroundImages.end())
        {
            auto last = backgroundImages.rbegin();
            selectedBackground = last->second;
            selectedScale = last->first;
        }
        else
        {
            selectedBackground = l->second;
            selectedScale = l->first;
        }
        painter->scale(selectedScale,selectedScale);

        painter->drawPixmap(0,0,selectedBackground);
    }
    painter->restore();

}

QPixmap BackgroundRaster::topLevelPixmap() const
{
    auto ret = backgroundImages.cbegin();
    return ret->second;
}

QString const &BackgroundRaster::filename() const
{
    return m_filename;
}

void BackgroundRaster::write(QJsonObject &json) const
{
    json["type"] = "BackgroundRaster";
    json["filename"] = m_filename;
}

void BackgroundRaster::writeToMissionPlan(QJsonArray& navArray) const
{
}


void BackgroundRaster::read(const QJsonObject &json)
{

}

qreal BackgroundRaster::pixelSize() const
{
    return m_pixel_size;
}

qreal BackgroundRaster::scaledPixelSize() const
{
    return m_pixel_size/m_map_scale;
}

void BackgroundRaster::updateMapScale(qreal scale)
{
    m_map_scale = scale;
}

qreal BackgroundRaster::mapScale() const
{
    return m_map_scale;
}

bool BackgroundRaster::canAcceptChildType(const std::string& childType) const
{
    return false;
}

float BackgroundRaster::getDepth(int x, int y) const
{
    if(depthValid() && x >= 0 && x < m_width && y >= 0 && y < m_height)
        return m_depth_data[y*m_width+x];
    return nan("");
}

float BackgroundRaster::getDepth(QGeoCoordinate const &location) const
{
    auto index = geoToPixel(location);
    return getDepth(index.x(), index.y());
}
