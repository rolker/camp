#ifndef LINESTRING_H
#define LINESTRING_H

#include "geographicsitem.h"
#include "group.h"

class Point;

class LineString : public Group, public GeoGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit LineString(QObject *parent = 0, QGraphicsItem *parentItem =0);
    
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QPainterPath shape() const override;
    
    void write(QJsonObject &json) const override;
    void read(const QJsonObject &json) override;
    
    QStandardItem * createItem(const QString & label) override;

    Point * addPoint(QGeoCoordinate const &location);    
    QList<Point *> points() const;

public slots:
    void updateProjectedPoints();

private:
    
    
};

#endif // LINESTRING_H
