#ifndef MISSIONITEM_H
#define MISSIONITEM_H

#include <QObject>

class MissionItem : public QObject
{
    Q_OBJECT
public:
    explicit MissionItem(QObject *parent = 0);

    virtual void write(QJsonObject &json) const = 0;
    virtual void read(const QJsonObject &json) = 0;

signals:

public slots:
};

#endif // MISSIONITEM_H
