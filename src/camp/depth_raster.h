#ifndef DEPTH_RASTER_H
#define DEPTH_RASTER_H

#include "georeferenced.h"
#include <QString>
#include <vector>

// Standalone depth-data provider extracted from BackgroundRaster (#59 PR6
// BackgroundRaster retirement, increment 1). Loads only the Float32 depth band
// of a georeferenced raster and answers depth queries by geo position — depth
// is data, not display, so this carries no QGraphicsItem / scene role. The load
// mirrors BackgroundRaster's exactly (same band selection, same GF_Read, same
// Georeferenced geo→pixel) so getDepth() values are unchanged; the chart image
// is drawn separately by camp::raster::RasterLayer.
class DepthRaster : public Georeferenced
{
public:
    explicit DepthRaster(const QString& filename);

    bool depthValid() const;
    float getDepth(int x, int y) const;
    float getDepth(const QGeoCoordinate& location) const;

    int width() const { return m_width; }
    int height() const { return m_height; }

    // [#59 ADR-0003] The source file, so the owner can match this provider to
    // its chart when that chart is removed (depth is a per-chart entry in the
    // provider list).
    const QString& filename() const { return m_filename; }

private:
    QString m_filename;
    int m_width = 0;
    int m_height = 0;
    std::vector<float> m_depth_data;
};

#endif // DEPTH_RASTER_H
