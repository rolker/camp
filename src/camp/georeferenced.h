#ifndef GEOREFERENCED_H
#define GEOREFERENCED_H

#include <QPointF>
#include <QGeoCoordinate>
class GDALDataset;
class OGRCoordinateTransformation;

class Georeferenced
{
public:
    Georeferenced();
    // [#152] Owns m_project/unprojectTransformation (created in
    // extractGeoreference) and frees them here. virtual is defensive
    // future-proofing: today neither subclass is deleted through a
    // Georeferenced* (DepthRaster is deleted as DepthRaster*, VectorDataset via
    // the QObject parent chain), so the dtor runs regardless — but making it
    // virtual keeps delete-through-base correct if that ever changes. The
    // =delete rule-of-three guards against a copy silently double-freeing those
    // owned handles; both subclasses are heap-only today, so this removes a
    // latent foot-gun without affecting any live call site.
    virtual ~Georeferenced();
    Georeferenced(const Georeferenced &) = delete;
    Georeferenced &operator=(const Georeferenced &) = delete;
    QPointF pixelToProjectedPoint(QPointF const &point) const;
    QPointF projectedPointToPixel(QPointF const &point) const;
    QPointF project(QGeoCoordinate const &point) const;
    QGeoCoordinate unproject(QPointF const &point) const;
    QPointF geoToPixel(QGeoCoordinate const &point) const;
    QGeoCoordinate pixelToGeo(QPointF const &point) const;
    QString const &projection() const;
protected:
    void extractGeoreference(GDALDataset *dataset);
private:
    double m_geoTransform[6];
    double m_inverseGeoTransform[6];
    OGRCoordinateTransformation *m_projectTransformation,*m_unprojectTransformation;
    QString m_projection;
};

#endif // GEOREFERENCED_H
