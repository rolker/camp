#include "depth_raster.h"
#include <gdal_priv.h>
#include <cmath>

DepthRaster::DepthRaster(const QString& filename):
    m_filename(filename)
{
    GDALDataset* dataset = reinterpret_cast<GDALDataset*>(GDALOpen(filename.toStdString().c_str(), GA_ReadOnly));
    if(!dataset)
        return;

    extractGeoreference(dataset);
    m_width = dataset->GetRasterXSize();
    m_height = dataset->GetRasterYSize();

    // The first Float32 band is the depth band (mirrors BackgroundRaster's
    // detection). Other bands are colour and belong to the displayed chart.
    for(int bandNumber = 1; bandNumber <= dataset->GetRasterCount(); bandNumber++)
    {
        GDALRasterBand* band = dataset->GetRasterBand(bandNumber);
        if(band->GetRasterDataType() == GDT_Float32 && !depthValid())
        {
            m_depth_data.resize(static_cast<size_t>(m_width) * m_height);
            if(band->RasterIO(GF_Read, 0, 0, m_width, m_height,
                              &m_depth_data.front(), m_width, m_height, GDT_Float32, 0, 0) != CE_None)
                m_depth_data.clear();
        }
    }

    GDALClose(dataset);
}

bool DepthRaster::depthValid() const
{
    // [#59] A Float32 band alone is not enough: without a projection the
    // georeference is meaningless, so getDepth(geo) would map every query through
    // a degenerate transform and return a *finite* bogus depth instead of NaN —
    // garbage that would feed the shoal-avoidance A*. Require a real projection
    // so a non-georeferenced raster is treated as "no depth" (getDepth → NaN =
    // safe unknown). Chart-derived depth rasters (KAP/BAG/VRT) always have one.
    return m_width > 0 && m_height > 0 && !projection().isEmpty()
           && m_depth_data.size() == static_cast<size_t>(m_width) * m_height;
}

float DepthRaster::getDepth(int x, int y) const
{
    if(depthValid() && x >= 0 && x < m_width && y >= 0 && y < m_height)
        return m_depth_data[static_cast<size_t>(y) * m_width + x];
    return std::nanf("");
}

float DepthRaster::getDepth(const QGeoCoordinate& location) const
{
    if(!depthValid())   // no depth band loaded → don't consult the georeference
        return std::nanf("");
    auto index = geoToPixel(location);
    return getDepth(index.x(), index.y());
}
