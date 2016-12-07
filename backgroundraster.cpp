#include "backgroundraster.h"
#include <QPainter>
#include <gdal_priv.h>

BackgroundRaster::BackgroundRaster(const QString &fname, QObject *parent, QGraphicsItem *parentItem) : QObject(parent), QGraphicsItem(parentItem)
{
    GDALDataset * dataset = reinterpret_cast<GDALDataset*>(GDALOpen(fname.toStdString().c_str(),GA_ReadOnly));
    if (dataset)
    {
        extractGeoreference(dataset);

        int width = dataset->GetRasterXSize();
        int height = dataset->GetRasterYSize();


        GDALRasterBand * band = dataset->GetRasterBand(1);

        GDALColorTable *colorTable = band->GetColorTable();

        std::vector<uint32_t> buffer(width);

        QImage image(width,height,QImage::Format_ARGB32);

        for(int j = 0; j<height; ++j)
        {
            band->RasterIO(GF_Read,0,j,width,1,&buffer.front(),width,1,GDT_UInt32,0,0);
            uchar *scanline = image.scanLine(j);
            for(int i = 0; i < width; ++i)
            {
                GDALColorEntry const *ce = colorTable->GetColorEntry(buffer[i]);
                scanline[i*4] = ce->c3;
                scanline[i*4+1] = ce->c2;
                scanline[i*4+2] = ce->c1;
                scanline[i*4+3] = ce->c4;
            }
        }
        backgroundImages[1] = QPixmap::fromImage(image);
        for(int i = 2; i < 128; i*=2)
        {
            backgroundImages[i] = QPixmap::fromImage(image.scaledToWidth(width/float(i),Qt::SmoothTransformation));
        }
    }
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
