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

    virtual bool readGeoJson(const QJsonObject &json);
    virtual void readGeoJsonProperties(const QJsonObject& json);

    virtual void writeToGeoJson(QJsonArray &array) const;
    virtual void writeGeoJson(QJsonObject &json, QString name = "") const;
    
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
        auto project = autonomousVehicleProject();
        if(project)
        {
            ret->setSpeed(project->speed());
            ret->setThrottle(project->throttle());
        }
        return ret;
    }
    
    virtual bool canAcceptChildType(std::string const &childType) const;
    virtual QList<QList<QGeoCoordinate> > getLines() const;

    /// Returns speed in knots
    double speed() const;

    /// Sets speed in knots
    void setSpeed(double speed);

    /// Returns throttle value (0.0 to 1.0)
    double throttle() const;

    /// Sets throttle value (0.0 to 1.0)
    void setThrottle(double throttle);

    int priority() const;
    void setPriority(int priority);

    const std::string& taskData() const;
    void setTaskData(const std::string& data);

signals:
    void speedChanged();
    void throttleChanged();

public slots:
    virtual void updateProjectedPoints();

protected:
    double m_speed = 0.0; //knots
    double throttle_ = 0.4; // 0.0 to 1.0

    /// Task priority, higher number is lower
    /// priority
    int m_priority = 0;

    std::string task_data_;

    
private:
    QList<MissionItem *> m_childrenMissionItems;

};

#endif // MISSIONITEM_H
