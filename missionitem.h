#ifndef MISSIONITEM_H
#define MISSIONITEM_H

#include <QObject>
#include <QStandardItem>

class AutonomousVehicleProject;

class MissionItem : public QObject
{
    Q_OBJECT
public:
    explicit MissionItem(QObject *parent = 0);

    virtual void write(QJsonObject &json) const = 0;
    virtual void read(const QJsonObject &json) = 0;
    
    AutonomousVehicleProject *autonomousVehicleProject() const;

    //void setItem(QStandardItem * item);
    QStandardItem * item() const;
    virtual QStandardItem * createItem(QString const &label) = 0;
    

signals:

public slots:

protected:
    template<typename T> QStandardItem * createItemDetails(QString const &label)
    {
        m_item = new QStandardItem(label);
        m_item->setData(QVariant::fromValue<T*>((T*)this));
        return m_item;
    }

    
private:
    QStandardItem *m_item;

};

#endif // MISSIONITEM_H
