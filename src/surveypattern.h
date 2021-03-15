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

    enum Alignment
    { 
        start,  // first line starts at start corner
        center, // lines are centered between start edge and end edge
        finish  // last line is along edge with end corner
    };
    
    int type() const override {return SurveyPatternType;}
    
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPainterPath shape() const override;

    void write(QJsonObject &json) const override;
    void writeToMissionPlan(QJsonArray & navArray) const override;
    void read(const QJsonObject &json) override;
    
    QGeoCoordinate const &startLocation() const;
    Waypoint * startLocationWaypoint() const;
    Waypoint * endLocationWaypoint() const;
    void setStartLocation(QGeoCoordinate const &location);
    void setEndLocation(QGeoCoordinate const &location, bool calc = true);
    void setSpacingLocation(QGeoCoordinate const &location, bool calc = true);

    bool hasSpacingLocation() const;

    double spacing() const;
    double direction() const;
    Alignment alignment() const;
    double lineLength() const;
    double totalWidth() const;
    int arcCount() const;
    double maxSegmentLength() const;

    void setDirectionAndSpacing(double direction, double spacing);
    void setAlignment(Alignment alignment);
    void setLineLength(double lineLength);
    void setTotalWidth(double totalWidth);

    QList<QList<QGeoCoordinate> > getLines() const override;
    
signals:
    void surveyPatternUpdated();

public slots:
    void waypointHasChanged(Waypoint *wp);
    void waypointAboutToChange();
    void updateProjectedPoints();
    void reverseDirection();

protected:
    Waypoint * createWaypoint();
    void updateEndLocation();

private:
    Waypoint * m_startLocation;
    Waypoint * m_endLocation;
    double m_lineLength;
    double m_totalWidth;
    double m_spacing;
    double m_direction;
    Alignment m_alignment;

    Waypoint * m_spacingLocation;

    bool m_internalUpdateFlag;

    void calculateFromWaypoints();

};

#endif // SURVEYPATTERN_H
