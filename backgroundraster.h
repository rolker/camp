#ifndef BACKGROUNDRASTER_H
#define BACKGROUNDRASTER_H

#include <QObject>
#include <QGraphicsItem>
#include <QPixmap>
#include "georeferenced.h"

class QPainter;

class BackgroundRaster: public QObject, public QGraphicsItem, public Georeferenced
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

private:
    typedef std::map<int,QPixmap> Mipmaps;
    Mipmaps backgroundImages;
    QString m_filename;

};

#endif // BACKGROUNDRASTER_H
