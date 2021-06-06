#ifndef RADARDISPLAY_H
#define RADARDISPLAY_H

#include <QObject>
#include "geographicsitem.h"
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <deque>
#include <ros/ros.h>

Q_DECLARE_METATYPE(QImage*)
Q_DECLARE_METATYPE(ros::Time)

class ROSLink;
class QOffscreenSurface;
class QOpenGLContext;
class QOpenGLFramebufferObject;
class QOpenGLShaderProgram;

namespace tf2_ros
{
    class Buffer;
}

class RadarDisplay : public QObject, public GeoGraphicsItem,  protected QOpenGLFunctions
{
    Q_OBJECT
    Q_INTERFACES(QGraphicsItem)
public:
    RadarDisplay(ROSLink* parent);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    int type() const override {return RadarDisplayType;}
    void setPixelSize(double s);
    void setTF2Buffer(tf2_ros::Buffer *buffer);
    void setMapFrame(std::string mapFrame);
    const QColor& getColor() const;
    
public slots:
    void addSector(double angle1, double angle2, double range, QImage *sector, ros::Time stamp, QString frame_id);
    void showRadar(bool show);
    void setColor(QColor color);

private:
    void initializeGL();
    
    struct Sector
    {
        Sector():angle1(0),angle2(0),range(0),heading(-1.0),sectorImage(nullptr),sectorTexture(nullptr)
        {}
        
        ros::Time timestamp;
        QString frame_id;
        double angle1, angle2, range, half_scanline_angle;
        double heading;
        QImage *sectorImage;
        QOpenGLTexture *sectorTexture;
    };
    
    double m_pixel_size = 1.0;

    std::deque<Sector> m_sectors;
    
    QOpenGLShaderProgram *m_program;
    QOffscreenSurface* m_surface = nullptr;
    QOpenGLContext* m_context = nullptr;
    QOpenGLFramebufferObject* m_fbo = nullptr;
    QOpenGLBuffer m_vbo;
    
    bool m_show_radar = true;
    QColor m_color ={0,255,0,255};

    tf2_ros::Buffer* m_tf_buffer = nullptr;
    std::string m_mapFrame;
};

#endif

