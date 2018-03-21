#include "group.h"

#include<QJsonObject>
#include<QJsonArray>
#include"autonomousvehicleproject.h"
#include "waypoint.h"
#include "trackline.h"
#include "surveypattern.h"
#include "platform.h"

Group::Group(QObject* parent):MissionItem(parent)
{

}

void Group::write(QJsonObject& json) const
{
    MissionItem::write(json);
    json["type"] = "Group";
    
    QJsonArray childrenArray;
    for(MissionItem *item: childMissionItems())
    {
        QJsonObject miObject;
        item->write(miObject);
        childrenArray.append(miObject);
    }
    
    json["children"] = childrenArray;
}

void Group::read(const QJsonObject& json)
{
    auto project = autonomousVehicleProject();
    QJsonArray childrenArray = json["children"].toArray();
    for (int childIndex = 0; childIndex < childrenArray.size(); ++childIndex)
    {
        QJsonObject object = childrenArray[childIndex].toObject();
        if(object["type"] == "BackgroundRaster")
            project->openBackground(object["filename"].toString());
        MissionItem *item = nullptr;
        if(object["type"] == "Waypoint")
            item = project->createWaypoint();
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

void Group::updateProjectedPoints()
{
    for(auto child: childMissionItems())
        child->updateProjectedPoints();
}
