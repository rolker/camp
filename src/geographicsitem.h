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
    enum {  BackgroundRasterType = UserType+1,
            WaypointType,
            TrackLineType,
            SurveyPatternType,
            PointType,
            LineStringType,
            PolygonType,
            ROSLinkType,
            SurveyAreaType,
            MeasuringToolType,
            RadarDisplayType
    };
    
    GeoGraphicsItem(QGraphicsItem *parentItem = Q_NULLPTR);

    
    QPointF geoToPixel(QGeoCoordinate const &point, AutonomousVehicleProject *p) const;
    QGeoCoordinate pixelToGeo(QPointF const &point) const;

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
