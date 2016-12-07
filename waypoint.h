#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <QObject>
#include <QGraphicsItem>
#include <QGeoCoordinate>

class Waypoint : public QObject, public QGraphicsItem
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    explicit Waypoint(QObject *parent = 0, QGraphicsItem *parentItem =0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    QGeoCoordinate const &location() const;
    void setLocation(QGeoCoordinate const &location);
signals:

public slots:

private:
    QGeoCoordinate m_location;
};

#endif // WAYPOINT_H
