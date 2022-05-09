#ifndef SEARCHPATTERN_H
#define SEARCHPATTERN_H

#include "geographicsmissionitem.h"

class Waypoint;

class SearchPattern : public GeoGraphicsMissionItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public:
  SearchPattern(MissionItem *parent = 0, int row = -1);

  enum Direction
  { 
    clockwise,
    counterclockwise
  };

  int type() const override {return SearchPatternType;}

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

  void setFirstLeg(double heading, double distance);

  bool canBeSentToRobot() const override;
  bool hasSpacingLocation() const;

  Direction direction() const;
  void setDirection(Direction direction);

  QList<QList<QGeoCoordinate> > getLines() const override;

signals:
  void searchPatternUpdated();

public slots:
  void waypointHasChanged(Waypoint *wp);
  void waypointAboutToChange();
  void updateProjectedPoints();
  void switchDirection();
  virtual void updateETE();

protected:
  Waypoint * createWaypoint();
  void updateEndLocation();

private:
  void calculateFromWaypoints();

  Waypoint * m_startLocation = nullptr;
  Waypoint * m_endLocation = nullptr;
  Waypoint * m_spacingLocation = nullptr;

  Direction m_direction = Direction::counterclockwise;
  double m_firstLegHeading = 180.0;
  double m_firstLegDistance = 0.0;
  double m_searchRadius = 0.0;

  bool m_internalUpdateFlag =  false;

};

#endif
