#ifndef SURVEYAREA_H
#define SURVEYAREA_H

#include "geographicsmissionitem.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

class SurveyArea : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
    
public:
    explicit SurveyArea(MissionItem *parent = 0);
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;
    
    Waypoint * createWaypoint();
    Waypoint * addWaypoint(QGeoCoordinate const &location);
    void removeWaypoint(Waypoint *wp);
    
    QList<Waypoint *> waypoints() const;

    void write(QJsonObject &json) const override;
    void writeToMissionPlan(QJsonArray & navArray) const override;
    void read(const QJsonObject &json) override;
    
    int type() const override {return SurveyAreaType;}
    
    bool canAcceptChildType(const std::string & childType) const override;
    
signals:
    
public slots:
    void updateProjectedPoints();
    void generateAdaptiveTrackLines();
    
private:
    typedef boost::geometry::model::d2::point_xy<double> BPoint;
    typedef boost::geometry::model::multi_point<BPoint> BMultiPoint;
    typedef boost::geometry::model::segment<BPoint> BSegment;
    typedef boost::geometry::model::linestring<BPoint> BLineString;
    typedef boost::geometry::model::polygon<BPoint> BPolygon;
    typedef boost::geometry::model::multi_linestring<BLineString> BMultiLineString;

    std::vector<QGeoCoordinate> generateNextLine(std::vector<QGeoCoordinate> const &guidePath, BackgroundRaster const &depthRaster, double tanHalfSwath, int side, BPolygon const &area_poly, double stepSize, BMultiLineString const & previousLines);
};

#endif
