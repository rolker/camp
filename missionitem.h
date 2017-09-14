#ifndef MISSIONITEM_H
#define MISSIONITEM_H

#include <QObject>

class AutonomousVehicleProject;
class QStandardItem;

class MissionItem : public QObject
{
    Q_OBJECT
public:
    explicit MissionItem(QObject *parent = 0);

    virtual void write(QJsonObject &json) const = 0;
    virtual void read(const QJsonObject &json) = 0;
    
    AutonomousVehicleProject *autonomousVehicleProject() const;

    QStandardItem * item() const;
    QStandardItem * createItem(QString const &label);
    

public slots:
    virtual void updateProjectedPoints();

public slots:

protected:

    
private:
    QStandardItem *m_item;

};

#endif // MISSIONITEM_H
