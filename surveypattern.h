#ifndef SURVEYPATTERN_H
#define SURVEYPATTERN_H

#include "geographicsitem.h"

class Waypoint;

class SurveyPattern : public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)

public:
    SurveyPattern(QObject *parent = 0, QGraphicsItem *parentItem =0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const;

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);

    QGeoCoordinate const &startLocation() const;
    Waypoint * startLocationWaypoint() const;
    Waypoint * endLocationWaypoint() const;
    void setStartLocation(QGeoCoordinate const &location);
    void setEndLocation(QGeoCoordinate const &location);
    void setSpacingLocation(QGeoCoordinate const &location, bool calc = true);

    bool hasSpacingLocation() const;

    double spacing() const;
    double direction() const;
    int arcCount() const;

    void setDirectionAndSpacing(double direction, double spacing);
    void setArcCount(int ac);

    QList<QGeoCoordinate> getPath() const;

signals:
    void surveyPatternUpdated();

public slots:
    void waypointHasChanged();
    void waypointAboutToChange();

protected:
    Waypoint * createWaypoint();

private:
    Waypoint * m_startLocation;
    Waypoint * m_endLocation;
    double m_spacing;
    double m_direction;
    int m_arcCount;

    Waypoint * m_spacingLocation;



};

#endif // SURVEYPATTERN_H
