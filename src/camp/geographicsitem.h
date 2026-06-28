#ifndef GEOGRAPHICSITEM_H
#define GEOGRAPHICSITEM_H

#include <QGraphicsItem>
#include <QGeoCoordinate>

class AutonomousVehicleProject;

class GeoGraphicsItem : public QGraphicsItem
{
    Q_INTERFACES(QGraphicsItem)

public:
    enum
    {
        // [#59 ADR-0003] BackgroundRasterType retired with BackgroundRaster.
        WaypointType = UserType+1,
        TrackLineType,
        SurveyPatternType,
        PointType,
        LineStringType,
        PolygonType,
        ROSLinkType,
        SurveyAreaType,
        MeasuringToolType,
        AISContactType,
        PlatformType,
        NavSourceType,
        SearchPatternType,
        GridType,
        AvoidAreaType,
        CollisionMonitorType,
        FootprintType,
        RunningTaskType,
    };
    
    GeoGraphicsItem(QGraphicsItem *parentItem = Q_NULLPTR);

    
    // [#59 ADR-0003] Chart-independent position: the scene is Web Mercator, so
    // this needs no BackgroundRaster. The old bg/AVP overloads are retired — every
    // call site now uses this single overload.
    QPointF geoToPixel(QGeoCoordinate const &point) const;
    QGeoCoordinate pixelToGeo(QPointF const &point) const;

    // [#59 PR6] Real metres covered by one display pixel at the given location,
    // from the active view's zoom and the Web-Mercator cos(latitude) correction
    // — the chart-independent replacement for BackgroundRaster::scaledPixelSize().
    // Sizes on-screen vessel/contact icons to a constant pixel footprint
    // regardless of zoom or whether a chart is loaded. Returns a sane default
    // (metres-per-unit at the location) when no view is attached yet.
    qreal metresPerPixel(QGeoCoordinate const &at) const;

    void prepareGeometryChange();

    bool showLabelFlag() const;
    void setShowLabelFlag(bool show=true);
    void setLabel(QString const &label);
    void setLabelPosition(QPointF pos);
    
    int type() const override=0;

private:
    QGraphicsSimpleTextItem *m_label;
    QString m_labelText;
    bool m_showLabelFlag;
    
};

Q_DECLARE_METATYPE(GeoGraphicsItem*)

#endif // GEOGRAPHICSITEM_H
