#include "missionitem.h"
#include "autonomousvehicleproject.h"
#include <QJsonObject>
#include <QJsonArray>
#include "waypoint.h"
#include "trackline.h"
#include "surveypattern.h"
#include "backgroundraster.h"
#include "behavior.h"
#include "surveyarea.h"
#include "group.h"
#include <QDebug>

MissionItem::MissionItem(QObject *parent, int row) : QObject(parent)
{
    MissionItem * parentMissionItem = qobject_cast<MissionItem*>(parent);
    if(parentMissionItem)
    {
        if(row < 0)
            parentMissionItem->m_childrenMissionItems.append(this);
        else
            parentMissionItem->m_childrenMissionItems.insert(row, this);
    }
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
    json["speed"]=m_speed;
}


void MissionItem::read(const QJsonObject& json)
{
    QString label = json["label"].toString();
    if(label.size() > 0)
        setObjectName(label);
    m_speed = json["speed"].toDouble();
}

void MissionItem::readChildren(const QJsonArray& json, int row)
{
    qDebug() << objectName() << " readChilden row: " << row;
    qDebug() << "  before:";
    for(auto c: m_childrenMissionItems)
        qDebug() << "      " << c->objectName();

    qDebug() << json;

    auto project = autonomousVehicleProject();
    for (int childIndex = 0; childIndex < json.size(); ++childIndex)
    {
        QJsonObject object = json[childIndex].toObject();
        qDebug() << object;
        if(object["type"] == "BackgroundRaster")
        {
            BackgroundRaster* bgr = project->openBackground(object["filename"].toString(), object["label"].toString());
            if(bgr)
                bgr->read(object);
        }
        if(object["type"] == "VectorDataset")
            project->openGeometry(object["filename"].toString());
        MissionItem *item = nullptr;
        int insertRow = row;
        if(object["type"] == "Waypoint")
            item = createMissionItem<Waypoint>(object["label"].toString(), insertRow);
        if(object["type"] == "TrackLine")
            item = project->createTrackLine(this, insertRow, object["label"].toString());
        if(object["type"] == "SurveyPattern")
            item = project->createSurveyPattern(this, insertRow, object["label"].toString());
        if(object["type"] == "SurveyArea")
            item = project->createSurveyArea(this, insertRow, object["label"].toString());
        if(object["type"] == "Group")
            item = project->createGroup(this, insertRow, object["label"].toString());
        if(item)
        {
            item->read(object);
            if(insertRow >= 0)
                insertRow++;
        }
    }
    qDebug() << "  after:";
    for(auto c: m_childrenMissionItems)
        qDebug() << "      " << c->objectName();

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

double MissionItem::speed() const
{
    return m_speed;
}

void MissionItem::setSpeed(double speed)
{
    m_speed = speed;
    emit speedChanged();
}
