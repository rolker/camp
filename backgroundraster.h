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
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QPixmap topLevelPixmap() const;
    QString const &filename() const;

    void write(QJsonObject &json) const override;
    void writeToMissionPlan(QJsonArray & navArray) const override;
    void read(const QJsonObject &json);
    
    qreal pixelSize() const;
    qreal scaledPixelSize() const;
    qreal mapScale() const;
    
    bool canAcceptChildType(const std::string & childType) const override;
    
    bool valid() const;
    bool depthValid() const;

    float getDepth(int x, int y) const;
    float getDepth(QGeoCoordinate const &location) const;
    
    int width() const {return m_width;}
    int height() const {return m_height;}
public slots:
    void updateMapScale(qreal scale); 

private:
    typedef std::map<int,QPixmap> Mipmaps;
    Mipmaps backgroundImages;
    QString m_filename;
    qreal m_pixel_size; // size of a pixel in meters.
    qreal m_map_scale;
    bool m_valid;

    int m_width;
    int m_height;
    std::vector<float> m_depth_data;

};

#endif // BACKGROUNDRASTER_H
