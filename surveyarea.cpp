#include "surveyarea.h"
#include "waypoint.h"
#include <QPainter>

SurveyArea::SurveyArea(MissionItem *parent) :GeoGraphicsMissionItem(parent)
{
}

QRectF SurveyArea::boundingRect() const
{
    return childrenBoundingRect();
}

void SurveyArea::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    painter->save();

    QPen p;
    p.setColor(Qt::blue);
    p.setCosmetic(true);
    p.setWidth(2);
    painter->setPen(p);
    painter->drawPath(shape());   
    painter->restore();
}

QPainterPath SurveyArea::shape() const
{
    auto children = waypoints();
    if (children.length() > 1)
    {
        auto i = children.begin();
        QPainterPath ret((*i)->pos());
        i++;
        while(i != children.end())
        {
            ret.lineTo((*i)->pos());
            i++;
        }
        ret.lineTo(children.front()->pos());
        QPainterPathStroker pps;
        pps.setWidth(5);
        return pps.createStroke(ret);

    }
    return QGraphicsItem::shape();
}

Waypoint * SurveyArea::createWaypoint()
{
    int i = childMissionItems().size();
    QString wplabel = "waypoint"+QString::number(i);
    Waypoint *wp = createMissionItem<Waypoint>(wplabel);

    wp->setFlag(QGraphicsItem::ItemIsMovable);
    wp->setFlag(QGraphicsItem::ItemIsSelectable);
    wp->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
    return wp;
}

Waypoint * SurveyArea::addWaypoint(const QGeoCoordinate& location)
{
    Waypoint *wp = createWaypoint();
    wp->setLocation(location);
    update();
    return wp;
}

void SurveyArea::removeWaypoint(Waypoint* wp)
{
    autonomousVehicleProject()->deleteItem(wp);
}

QList<Waypoint *> SurveyArea::waypoints() const
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

QList<QList<QGeoCoordinate> > SurveyArea::getLines() const
{
}

void SurveyArea::write(QJsonObject& json) const
{
}

void SurveyArea::writeToMissionPlan(QJsonArray& navArray) const
{
}

void SurveyArea::read(const QJsonObject& json)
{
}

void SurveyArea::updateProjectedPoints()
{
    for(auto wp: waypoints())
        wp->updateProjectedPoints();
}

bool SurveyArea::canAcceptChildType(const std::string& childType) const
{
    return childType == "Waypoint";
}
