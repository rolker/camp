#include "avoid_area.h"
#include <QPainter>
#include <QJsonObject>
#include <QJsonArray>
#include "backgroundraster.h"
#include <QDebug>
#include "waypoint.h"
#include <cmath>

AvoidArea::AvoidArea(MissionItem *parent, int row) :GeoGraphicsMissionItem(parent, row)
{

}

QRectF AvoidArea::boundingRect() const
{
  return shape().boundingRect().marginsAdded(QMarginsF(5,5,5,5));
}

void AvoidArea::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
  painter->save();

  QPen p;
  p.setColor(Qt::red);
  p.setCosmetic(true);
  p.setWidth(2);
  painter->setPen(p);

  QBrush b = painter->brush();
  b.setColor(QColor(255, 0, 0, 128));
  b.setStyle(Qt::BrushStyle::SolidPattern);
  painter->setBrush(b);

  painter->drawPath(shape());
  painter->restore();
}

QPainterPath AvoidArea::shape() const
{
  auto children = points();
  if (children.length() == 2)
  {
    QPainterPath ret;
    auto p0 = children[0]->pos();
    auto p1 = children[1]->pos();
    auto delta = p1-p0;
    auto distance = sqrt(delta.x()*delta.x()+delta.y()*delta.y());
    ret.addEllipse(p0, distance, distance);
    return ret;
  }

  if (children.length() > 2)
  {
    auto i = children.begin();
    QPainterPath ret((*i)->pos());
    auto last = i;
    i++;
    while(i != children.end())
    {
      ret.lineTo((*i)->pos());
      last = i;
      i++;
    }
    ret.lineTo(children.front()->pos());
    return ret;

  }
  return QGraphicsItem::shape();
}

Waypoint * AvoidArea::createPoint()
{
  prepareGeometryChange();
  int i = childMissionItems().size();
  QString wplabel = "point"+QString::number(i);
  Waypoint *wp = createMissionItem<Waypoint>(wplabel);
  
  connect(wp, &Waypoint::waypointMoved, autonomousVehicleProject(), &AutonomousVehicleProject::updateAvoidanceAreas);

  wp->setFlag(QGraphicsItem::ItemIsMovable);
  wp->setFlag(QGraphicsItem::ItemIsSelectable);
  wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
  return wp;
}

Waypoint * AvoidArea::addPoint(const QGeoCoordinate& location)
{
  Waypoint *wp = createPoint();
  wp->setLocation(location);
  update();
  return wp;
}

void AvoidArea::removePoint(Waypoint* wp)
{
  autonomousVehicleProject()->deleteItem(wp);
}

QList<Waypoint *> AvoidArea::points() const
{
  QList<Waypoint *> ret;
  auto children = childMissionItems();
  for(auto child: children)
  {
      Waypoint *wp = qobject_cast<Waypoint*>(child);
      if(wp)
          ret.append(wp);
  }
  return ret;
}

void AvoidArea::write(QJsonObject& json) const
{
  MissionItem::write(json);
  json["type"] = "AvoidArea";
}

void AvoidArea::writeToMissionPlan(QJsonArray& navArray) const
{
}

void AvoidArea::read(const QJsonObject& json)
{
  GeoGraphicsMissionItem::read(json);
}

void AvoidArea::updateProjectedPoints()
{
  for(auto p: points())
    p->updateProjectedPoints();
}

bool AvoidArea::canAcceptChildType(const std::string& childType) const
{
  if (childType == "Waypoint") return true;
  return false;
}

bool AvoidArea::canBeSentToRobot() const
{
  return false;
}

QVariant AvoidArea::itemChange(GraphicsItemChange change, const QVariant &value)
{
  if(change == ItemPositionChange || change == ItemScenePositionHasChanged)
    emit avoidAreaChanged();

  return QGraphicsItem::itemChange(change,value);
}
