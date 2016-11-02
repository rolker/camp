#ifndef BACKGROUNDRASTER_H
#define BACKGROUNDRASTER_H

#include <QObject>
#include <QPixmap>
#include "georeferenced.h"

class QPainter;
class ScaledView;

class BackgroundRaster: public QObject, public Georeferenced
{
    Q_OBJECT
public:
    BackgroundRaster(QObject *parent = 0, const QString &fname = QString());
    void paint(QPainter *painter, const ScaledView &view) const;

private:
    typedef std::map<int,QPixmap> Mipmaps;
    Mipmaps backgroundImages;

};

#endif // BACKGROUNDRASTER_H
