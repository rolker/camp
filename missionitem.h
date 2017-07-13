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

    void setItem(QStandardItem * item);
    QStandardItem * item() const;

signals:

public slots:

private:
    QStandardItem *m_item;

};

#endif // MISSIONITEM_H
