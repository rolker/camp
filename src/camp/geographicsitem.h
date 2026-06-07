#ifndef GEOGRAPHICSITEM_H
#define GEOGRAPHICSITEM_H

#include <QGraphicsItem>
#include <QGeoCoordinate>

class AutonomousVehicleProject;
class BackgroundRaster;

class GeoGraphicsItem : public QGraphicsItem
{
    Q_INTERFACES(QGraphicsItem)

public:
    enum
    {
        BackgroundRasterType = UserType+1,
        WaypointType,
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
    };
    
    GeoGraphicsItem(QGraphicsItem *parentItem = Q_NULLPTR);

    
    // [#59 PR6] Chart-independent position: the scene is Web Mercator, so this
    // needs no BackgroundRaster. The bg/AVP overloads remain for callers not yet
    // migrated; they delegate here (bg is ignored — see geoToPixel(point)).
    QPointF geoToPixel(QGeoCoordinate const &point) const;
    QPointF geoToPixel(QGeoCoordinate const &point, AutonomousVehicleProject *p) const;
    QPointF geoToPixel(QGeoCoordinate const &point, BackgroundRaster *bg) const;
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

protected:
    BackgroundRaster* findParentBackgroundRaster() const;

private:
    QGraphicsSimpleTextItem *m_label;
    QString m_labelText;
    bool m_showLabelFlag;
    
};

Q_DECLARE_METATYPE(GeoGraphicsItem*)

#endif // GEOGRAPHICSITEM_H
