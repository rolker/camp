#ifndef MISSIONITEM_H
#define MISSIONITEM_H

#include <QObject>
#include "autonomousvehicleproject.h"

class QStandardItem;
class QGraphicsItem;

class MissionItem : public QObject
{
    Q_OBJECT
public:
    explicit MissionItem(QObject *parent = 0, int row = -1);

    virtual void write(QJsonObject &json) const;
    virtual void writeToMissionPlan(QJsonArray &navArray) const = 0;
    virtual void writeBehaviorsToMissionPlanObject(QJsonObject &missionObject) const;
    virtual void read(const QJsonObject &json);
    virtual void readChildren(const QJsonArray &json, int row = -1);
    virtual bool canBeSentToRobot() const = 0;
    
    virtual QGraphicsItem *findParentGraphicsItem();
    
    AutonomousVehicleProject *autonomousVehicleProject() const;

    QList<MissionItem*> const &childMissionItems() const;
    void removeChildMissionItem(MissionItem *cmi);
    
    int row() const;
    
    template<typename T> T* createMissionItem(QString const &name = "", int row = -1)
    {
        AutonomousVehicleProject::RowInserter ri(*autonomousVehicleProject(),this, row);
        T* ret = new T(this, row);
        ret->setObjectName(name);
        return ret;
    }
    
    virtual bool canAcceptChildType(std::string const &childType) const;
    virtual QList<QList<QGeoCoordinate> > getLines() const;

public slots:
    virtual void updateProjectedPoints();

protected:

    
private:
    QList<MissionItem *> m_childrenMissionItems;

};

#endif // MISSIONITEM_H
