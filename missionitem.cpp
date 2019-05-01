#include "missionitem.h"
#include "autonomousvehicleproject.h"
#include <QJsonObject>
#include <QJsonArray>
#include "waypoint.h"
#include "trackline.h"
#include "surveypattern.h"
#include "platform.h"
#include "backgroundraster.h"
#include "behavior.h"

MissionItem::MissionItem(QObject *parent) : QObject(parent)
{
    MissionItem * parentMissionItem = qobject_cast<MissionItem*>(parent);
    if(parentMissionItem)
        parentMissionItem->m_childrenMissionItems.append(this);
}

AutonomousVehicleProject* MissionItem::autonomousVehicleProject() const
{
    QObject *o = parent();
    while(o)
    {
        AutonomousVehicleProject * avp = qobject_cast<AutonomousVehicleProject*>(o);
        if(avp)
            return avp;
        o = o->parent();
    }
    return nullptr;
}

void MissionItem::updateProjectedPoints()
{
}

const QList<MissionItem *> & MissionItem::childMissionItems() const
{
    return m_childrenMissionItems;
}

QList<QList<QGeoCoordinate> > MissionItem::getLines() const
{
    QList<QList<QGeoCoordinate> > ret;
    for(auto mi:m_childrenMissionItems)
        for(auto line: mi->getLines())
            ret.append(line);
    return ret;
}


int MissionItem::row() const
{
    MissionItem *item = qobject_cast<MissionItem*>(parent());
    if(item)
        return item->childMissionItems().indexOf(const_cast<MissionItem*>(this));
    return 0;
}

void MissionItem::write(QJsonObject& json) const
{
    json["label"] = objectName();
}


void MissionItem::read(const QJsonObject& json)
{
    setObjectName(json["label"].toString());
}

void MissionItem::readChildren(const QJsonArray& json)
{
    auto project = autonomousVehicleProject();
    for (int childIndex = 0; childIndex < json.size(); ++childIndex)
    {
        QJsonObject object = json[childIndex].toObject();
        if(object["type"] == "BackgroundRaster")
            project->openBackground(object["filename"].toString());
        if(object["type"] == "VectorDataset")
            project->openGeometry(object["filename"].toString());
        MissionItem *item = nullptr;
        if(object["type"] == "Waypoint")
            item = createMissionItem<Waypoint>("waypoint");
        if(object["type"] == "TrackLine")
            item = project->createTrackLine();
        if(object["type"] == "SurveyPattern")
            item = project->createSurveyPattern();
        if(object["type"] == "Platform")
            item = project->createPlatform();
        if(item)
            item->read(object);
    }

}

QGraphicsItem * MissionItem::findParentGraphicsItem()
{
    if(parent() == autonomousVehicleProject())
        return autonomousVehicleProject()->getBackgroundRaster();
    MissionItem *pmi = qobject_cast<MissionItem*>(parent());
    if(pmi)
        return pmi->findParentGraphicsItem();
    return nullptr;
}

bool MissionItem::canAcceptChildType(const std::string& childType) const
{
    return childType == "Behavior";
}

void MissionItem::removeChildMissionItem(MissionItem* cmi)
{
    m_childrenMissionItems.removeAll(cmi);
}

void MissionItem::writeBehaviorsToMissionPlanObject(QJsonObject& missionObject) const
{
    QJsonObject behaviorsObject;
    for(auto mi: m_childrenMissionItems)
    {
        Behavior *b = qobject_cast<Behavior*>(mi);
        if(b)
            b->writeToMissionPlanObject(behaviorsObject);
    }
    if(!behaviorsObject.empty())
        missionObject["behaviors"] = behaviorsObject;
}
