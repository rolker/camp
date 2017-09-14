#ifndef BACKGROUNDRASTER_H
#define BACKGROUNDRASTER_H

#include "missionitem.h"
#include <QGraphicsItem>
#include "georeferenced.h"
#include <QPixmap>

class QPainter;

class BackgroundRaster: public MissionItem, public QGraphicsItem, public Georeferenced
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    BackgroundRaster(const QString &fname = QString(), QObject *parent = 0, QGraphicsItem *parentItem =0);
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QPixmap topLevelPixmap() const;
    QString const &filename() const;

    void write(QJsonObject &json) const;
    void read(const QJsonObject &json);
    
private:
    typedef std::map<int,QPixmap> Mipmaps;
    Mipmaps backgroundImages;
    QString m_filename;

};

#endif // BACKGROUNDRASTER_H
