#ifndef RADARDISPLAY_H
#define RADARDISPLAY_H

#include <QObject>
#include "geographicsitem.h"
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <deque>

Q_DECLARE_METATYPE(QImage*)

class ROSLink;
class QOffscreenSurface;
class QOpenGLContext;
class QOpenGLFramebufferObject;
class QOpenGLShaderProgram;

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
    
public slots:
    void addSector(double angle1, double angle2, double range, QImage *sector);    
    
private:
    void initializeGL();
    
    struct Sector
    {
        Sector():angle1(0),angle2(0),range(0),sectorImage(nullptr),sectorTexture(nullptr)
        {}
        
        double angle1, angle2, range, half_scanline_angle;
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
};

#endif

