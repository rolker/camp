#ifndef RADARDISPLAY_H
#define RADARDISPLAY_H

#include <QObject>
#include "geographicsitem.h"
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QMutex>
#include <deque>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "marine_sensor_msgs/RadarSector.h"

Q_DECLARE_METATYPE(QImage*)
Q_DECLARE_METATYPE(ros::Time)

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
    RadarDisplay(QObject* parent = nullptr, QGraphicsItem *parentItem = nullptr);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    int type() const override {return RadarDisplayType;}
    void setPixelSize(double s);
    void setTF2Buffer(tf2_ros::Buffer *buffer);
    void setMapFrame(std::string mapFrame);
    const QColor& getColor() const;
    
public slots:
    void showRadar(bool show);
    void setColor(QColor color);
    void sectorAdded();
    void subscribe(QString topic);
    void updatePosition();

private:
    void initializeGL();
    void radarCallback(const marine_sensor_msgs::RadarSector::ConstPtr &message);
    void updateRadarImage();

    
    struct Sector
    {
        Sector():angle1(0),angle2(0),range(0),yaw(-1.0),sectorImage(nullptr),sectorTexture(nullptr)
        {}
        
        ros::Time timestamp;
        QString frame_id;
        double angle1, angle2, range, half_scanline_angle;
        double yaw;
        double have_yaw = false;
        double rendered = false;
        QImage *sectorImage;
        QOpenGLTexture *sectorTexture;
    };
    
    double m_pixel_size = 1.0;

    std::deque<Sector> m_sectors;

    std::deque<Sector> m_new_sectors;
    QMutex m_new_sectors_mutex;

    double m_range = 0.0;
    QMutex m_range_mutex;

    QOpenGLShaderProgram *m_program;
    QOffscreenSurface* m_surface = nullptr;
    QOpenGLContext* m_context = nullptr;
    QOpenGLFramebufferObject* m_fbo = nullptr;
    QOpenGLBuffer m_vbo;

    QImage m_radar_image;
    QMutex m_radar_image_mutex;

    bool m_opengl_initialized = false;
    
    bool m_show_radar = true;

    QColor m_color ={0,255,0,255};
    QMutex m_color_mutex;

    tf2_ros::Buffer* m_tf_buffer = nullptr;
    std::string m_mapFrame;
    std::string m_radar_frame;

    ros::CallbackQueue m_ros_queue;
    std::shared_ptr<ros::AsyncSpinner> m_spinner;
    ros::Subscriber m_subscriber;

    QThread* m_radarImageThread;
};

#endif

