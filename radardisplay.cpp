#include "radardisplay.h"

#include <cmath>
#include "autonomousvehicleproject.h"
#include "roslink.h"
#include <QOffscreenSurface>
#include <QOpenGLShader>
#include <QPainter>
#include <QOpenGLFramebufferObject>


RadarDisplay::RadarDisplay(ROSLink* parent): QObject(parent), GeoGraphicsItem(parent)
{
    
    QSurfaceFormat surfaceFormat;
    surfaceFormat.setMajorVersion(4);
    surfaceFormat.setMinorVersion(3);
    
    m_context = new QOpenGLContext();
    m_context->setFormat(surfaceFormat);
    m_context->create();
    if(!m_context->isValid()) return;

    m_surface = new QOffscreenSurface();
    m_surface->setFormat(surfaceFormat);
    m_surface->create();
    if(!m_surface->isValid()) return;

    m_context->makeCurrent(m_surface);
    QOpenGLFramebufferObjectFormat fboFormat;
    m_fbo = new QOpenGLFramebufferObject(2048,2048,fboFormat);

    initializeGL();

}

void RadarDisplay::setPixelSize(double s)
{
    m_pixel_size = s;
}

void RadarDisplay::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    
    QVector<GLfloat> vertData;
    vertData.append(-1.0); vertData.append(-1.0); vertData.append(0.0);
    vertData.append(1.0); vertData.append(-1.0); vertData.append(0.0);
    vertData.append(1.0); vertData.append(1.0); vertData.append(0.0);
    vertData.append(-1.0); vertData.append(1.0); vertData.append(0.0);
    
    m_vbo.create();
    m_vbo.bind();
    m_vbo.allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));
    
#define PROGRAM_VERTEX_ATTRIBUTE 0

    QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    const char *vsrc =
        "attribute highp vec4 vertex;\n"
        "varying mediump vec4 texc;\n"
        "uniform mediump mat4 matrix;\n"
        "void main(void)\n"
        "{\n"
        "    gl_Position = matrix * vertex;\n"
        "    texc = vertex;\n"
        "}\n";
    vshader->compileSourceCode(vsrc);

    QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
    const char *fsrc =
        "#define M_PI 3.1415926535897932384626433832795\n"
        "uniform sampler2D texture;\n"
        "varying mediump vec4 texc;\n"
        "uniform float minAngle;\n"
        "uniform float maxAngle;\n"
        "void main(void)\n"
        "{\n"
        "    if(texc.x == 0.0) discard;\n"
        "    float r = length(texc.xy);\n"
        "    if(r>1.0) discard;\n"
        "    float theta = atan(texc.x, texc.y);\n"
        "    if(minAngle > 0.0 && theta < 0.0) theta += 2.0*M_PI;\n"
        "    if(theta < minAngle) discard;\n"
        "    if(theta > maxAngle) discard;\n"
        "    vec4 radarData = texture2D(texture, vec2(r, (theta-minAngle)/(maxAngle-minAngle)));\n"
        "    gl_FragColor.ga = radarData.rg;\n"
        "}\n";
    fshader->compileSourceCode(fsrc);

    m_program = new QOpenGLShaderProgram;
    m_program->addShader(vshader);
    m_program->addShader(fshader);
    m_program->bindAttributeLocation("vertex", PROGRAM_VERTEX_ATTRIBUTE);
    m_program->link();

    m_program->bind();
    m_program->setUniformValue("texture", 0);
    
}

QRectF RadarDisplay::boundingRect() const
{
    if(!m_sectors.empty())
    {
        double r = m_sectors.back().range;
        r /= m_pixel_size;
        return QRectF(-r,-r,r*2,r*2);
    }
    return QRectF();
}

void RadarDisplay::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    if(!m_sectors.empty())
    {
        double r = m_sectors.back().range;
        qDebug() << "range: " << r;
        r /= m_pixel_size;
        QPen p;
        p.setColor(Qt::green);
        p.setWidth(2);
        painter->setPen(p);
        QRectF radarRect(-r,-r,r*2,r*2);
        painter->drawEllipse(radarRect);

        
        m_context->makeCurrent(m_surface);
        m_fbo->bind();
        glClear(GL_COLOR_BUFFER_BIT);

        QMatrix4x4 matrix;
        matrix.ortho(-1, 1, -1, 1, 4.0f, 15.0f);
        matrix.translate(0.0f, 0.0f, -10.0f);
        
        glViewport(0,0,2048,2048);
        
        m_program->setUniformValue("matrix", matrix);
        m_program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
        m_program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 3 * sizeof(GLfloat));
    
        while(m_sectors.size() > 75)
        {
            if(m_sectors.front().sectorImage)
                delete m_sectors.front().sectorImage;
            if(m_sectors.front().sectorTexture)
                delete m_sectors.front().sectorTexture;
            m_sectors.pop_front();
        }
        for(Sector &s: m_sectors)
        if(s.sectorImage)
        {
            if(!s.sectorTexture)
            {
                s.sectorTexture = new QOpenGLTexture(*s.sectorImage);
            }
            m_program->setUniformValue("minAngle", GLfloat(s.angle1-s.half_scanline_angle*1.1));
            m_program->setUniformValue("maxAngle", GLfloat(s.angle2+s.half_scanline_angle*1.1));
            s.sectorTexture->bind();
            glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
        }
        
        painter->drawImage(radarRect, m_fbo->toImage());
        
    }
    
}


void RadarDisplay::addSector(double angle1, double angle2, double range, QImage *sector)
{
    prepareGeometryChange();
    //std::cerr << angle1 << " - " << angle2 << " degs, " << range << " meters" << std::endl;
    Sector s;
    if(angle1 > angle2)
        s.angle1 = (angle1-360.0)*M_PI/180.0;
    else
        s.angle1 = angle1*M_PI/180.0;
    s.angle2 = angle2*M_PI/180.0;
    s.half_scanline_angle = (s.angle2 - s.angle1)/(2.0*sector->height());
    s.range = range;
    s.sectorImage = sector;
    m_sectors.push_back(s);
    update();
}
