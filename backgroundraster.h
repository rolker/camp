#ifndef BACKGROUNDRASTER_H
#define BACKGROUNDRASTER_H

#include <QObject>
#include <QPixmap>

class QPainter;

class BackgroundRaster: public QObject
{
    Q_OBJECT
public:
    BackgroundRaster(QObject *parent = 0, const QString &fname = QString());
    void paint(QPainter *painter, double scale, QPointF const &center, const QRect &viewRect) const;

private:
    typedef std::map<int,QPixmap> Mipmaps;
    Mipmaps backgroundImages;

};

#endif // BACKGROUNDRASTER_H
