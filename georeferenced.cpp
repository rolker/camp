#include "georeferenced.h"

#include <QtMath>
#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include <QDebug>

Georeferenced::Georeferenced(): m_geoTransform{0.0,1.0,0.0,0.0,0.0,1.0}, m_inverseGeoTransform{0.0,1.0,0.0,0.0,0.0,1.0}, m_projectTransformation(0), m_unprojectTransformation(0)
{

}

QPointF Georeferenced::pixelToProjectedPoint(const QPointF &point) const
{
    return QPointF(m_geoTransform[0]+point.x()*m_geoTransform[1]+point.y()*m_geoTransform[2],
                   m_geoTransform[3]+point.x()*m_geoTransform[4]+point.y()*m_geoTransform[5]);
}

QPointF Georeferenced::projectedPointToPixel(const QPointF &point) const
{
    return QPointF(m_inverseGeoTransform[0]+point.x()*m_inverseGeoTransform[1]+point.y()*m_inverseGeoTransform[2],
                   m_inverseGeoTransform[3]+point.x()*m_inverseGeoTransform[4]+point.y()*m_inverseGeoTransform[5]);
}

void Georeferenced::extractGeoreference(GDALDataset *dataset)
{
    dataset->GetGeoTransform(m_geoTransform);
    qDebug() << "geoTransform: " << m_geoTransform[0] << ", " << m_geoTransform[1] << ", " << m_geoTransform[2] << ", " << m_geoTransform[3] << ", " << m_geoTransform[4] << ", " << m_geoTransform[5];
    GDALInvGeoTransform(m_geoTransform,m_inverseGeoTransform);

    OGRSpatialReference projected, wgs84;

    qDebug() << "projection:" << dataset->GetProjectionRef();
    qDebug() << "gcp projection:" << dataset->GetGCPProjection();

    char * wktProjection = const_cast<char *>(dataset->GetProjectionRef());
    if(wktProjection[0] == 0)
        wktProjection = const_cast<char *>(dataset->GetGCPProjection());
    m_projection = wktProjection;
    projected.importFromWkt(&wktProjection);

    wgs84.SetWellKnownGeogCS("WGS84");

    m_unprojectTransformation = OGRCreateCoordinateTransformation(&projected,&wgs84);
    m_projectTransformation = OGRCreateCoordinateTransformation(&wgs84,&projected);
}

QPointF Georeferenced::project(const QGeoCoordinate &point) const
{
    if(m_projectTransformation)
    {
        double x = point.longitude();
        double y = point.latitude();
        m_projectTransformation->Transform(1,&x,&y);
        return QPointF(x,y);
    }
    return QPointF();
}

QGeoCoordinate Georeferenced::unproject(const QPointF &point) const
{
    if(m_unprojectTransformation)
    {
        double x = point.x();
        double y = point.y();
        m_unprojectTransformation->Transform(1,&x,&y);
        return QGeoCoordinate(y, x);
    }
    return QGeoCoordinate();
}

QPointF Georeferenced::geoToPixel(const QGeoCoordinate &point) const
{
    return projectedPointToPixel(project(point));
}

QGeoCoordinate Georeferenced::pixelToGeo(const QPointF &point) const
{
    return unproject(pixelToProjectedPoint(point));
}

QString const &Georeferenced::projection() const
{
    return m_projection;
}

