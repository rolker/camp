#ifndef SURVEYPATTERN_H
#define SURVEYPATTERN_H

#include "geographicsmissionitem.h"

class Waypoint;

class SurveyPattern : public GeoGraphicsMissionItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    SurveyPattern(MissionItem *parent = 0);

    
    int type() const override {return SurveyPatternType;}
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
    QGeoCoordinate const &startLocation() const;
    Waypoint * startLocationWaypoint() const;
    Waypoint * endLocationWaypoint() const;
    void setStartLocation(QGeoCoordinate const &location);
    void setEndLocation(QGeoCoordinate const &location, bool calc = true);
    void setSpacingLocation(QGeoCoordinate const &location, bool calc = true);

    bool hasSpacingLocation() const;

    double spacing() const;
    double direction() const;
    double lineLength() const;
    double totalWidth() const;
    int arcCount() const;
    double maxSegmentLength() const;

    void setDirectionAndSpacing(double direction, double spacing);
    void setLineLength(double lineLength);
    void setTotalWidth(double totalWidth);
    void setArcCount(int ac);
    void setMaxSegmentLength(double maxLength);

    //QList<QGeoCoordinate> getPath() const;
    QList<QList<QGeoCoordinate> > getLines() const;
    
signals:
    void surveyPatternUpdated();

public slots:
    void waypointHasChanged(Waypoint *wp);
    void waypointAboutToChange();
    void updateProjectedPoints();
    void onCurrentPlatformUpdated();

protected:
    Waypoint * createWaypoint();
    void updateLabel();
    void updateEndLocation();

private:
    Waypoint * m_startLocation;
    Waypoint * m_endLocation;
    double m_lineLength;
    double m_totalWidth;
    double m_spacing;
    double m_direction;
    int m_arcCount;
    double m_maxSegmentLength;

    Waypoint * m_spacingLocation;

    bool m_internalUpdateFlag;

    void calculateFromWaypoints();

};

#endif // SURVEYPATTERN_H
