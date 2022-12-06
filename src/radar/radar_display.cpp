#include "radar_display.h"

#include <cmath>
#include "autonomousvehicleproject.h"
#include <QOffscreenSurface>
#include <QOpenGLShader>
#include <QPainter>
#include <QOpenGLFramebufferObject>
#include <QThread>
#include <tf2/utils.h>
#include "gz4d_geo.h"
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>


RadarDisplay::RadarDisplay(QObject* parent, QGraphicsItem *parentItem): QObject(parent), GeoGraphicsItem(parentItem)
{
  m_radarImageThread = QThread::create(std::bind(&RadarDisplay::updateRadarImage, this));
  m_radarImageThread->start();
}

void RadarDisplay::setTF2Buffer(tf2_ros::Buffer* buffer)
{
    m_tf_buffer = buffer;
}

void RadarDisplay::setMapFrame(std::string mapFrame)
{
    m_mapFrame = mapFrame;
}

void RadarDisplay::setPixelSize(double s)
{
    m_pixel_size = s;
    ROS_INFO_STREAM("Pixel size: " << s);
}

void RadarDisplay::subscribe(QString topic)
{
    if(!m_spinner)
    {
        m_spinner = std::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1, &m_ros_queue));
        m_spinner->start();
    }
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<marine_sensor_msgs::RadarSector>(topic.toStdString(), 300, boost::bind(&RadarDisplay::radarCallback, this, _1), ros::VoidPtr(), &m_ros_queue);
    m_subscriber = ros::NodeHandle().subscribe(ops);
}

void RadarDisplay::initializeGL()
{
    QSurfaceFormat surfaceFormat;
    surfaceFormat.setMajorVersion(4);
    surfaceFormat.setMinorVersion(3);
    surfaceFormat.setAlphaBufferSize(8);
    surfaceFormat.setRedBufferSize(8);
    surfaceFormat.setGreenBufferSize(8);
    surfaceFormat.setBlueBufferSize(8);
    
    m_context = new QOpenGLContext();
    m_context->setFormat(surfaceFormat);
    m_context->create();
    if(!m_context->isValid())
    {
      ROS_ERROR_STREAM("OpenGL context not valid");
      return;
    }

    m_surface = new QOffscreenSurface();
    m_surface->setFormat(surfaceFormat);
    m_surface->create();
    if(!m_surface->isValid())
    {
      ROS_ERROR_STREAM("OpenGL surface not valid");
      return;
    } 

    m_context->makeCurrent(m_surface);

    QOpenGLFramebufferObjectFormat fboFormat;
    m_fbo = new QOpenGLFramebufferObject(2048,2048,fboFormat);
    if (!m_fbo->isValid())
    {
      ROS_ERROR_STREAM("OpenGL fbo not valid");
      return;
    } 
    

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

    QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, m_context);
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

    QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, m_context);
    const char *fsrc =
        "#define M_PI 3.1415926535897932384626433832795\n"
        "uniform sampler2D texture;\n"
        "varying mediump vec4 texc;\n"
        "uniform float minAngle;\n"
        "uniform float maxAngle;\n"
        "uniform float fade;\n"
        "uniform vec4 color;\n"
        "void main(void)\n"
        "{\n"
        "    if(texc.x == 0.0) discard;\n"
        "    float r = length(texc.xy);\n"
        "    if(r>1.0) discard;\n"
        "    float theta = atan(texc.y, texc.x);\n"
        "    if(minAngle > 0.0 && theta < 0.0) theta += 2.0*M_PI;\n"
        "    if(theta < minAngle) discard;\n"
        "    if(theta > maxAngle) discard;\n"
        "    vec4 radarData = texture2D(texture, vec2(r, (theta-minAngle)/(maxAngle-minAngle)));\n"
        "    if(radarData.r < 0.01) discard;\n"
        "    gl_FragColor = color*fade*radarData.r;\n"
        "    //gl_FragColor.a = radarData.r*fade;\n"
        "}\n";
    fshader->compileSourceCode(fsrc);

    m_program = new QOpenGLShaderProgram;
    m_program->addShader(vshader);
    m_program->addShader(fshader);
    m_program->bindAttributeLocation("vertex", PROGRAM_VERTEX_ATTRIBUTE);
    m_program->link();

    m_program->bind();
    m_program->setUniformValue("texture", 0);

    m_opengl_initialized = true;
}

QRectF RadarDisplay::boundingRect() const
{
  if (m_range > 0);
  {
    double r = m_range / m_pixel_size;
    return QRectF(-r,-r,r*2,r*2);
  }
  return QRectF();
}

void RadarDisplay::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    if(m_show_radar && !m_sectors.empty())
    {
        double r;
        {
          QMutexLocker lock(&m_range_mutex);
          r = m_range;
        }
        r /= m_pixel_size;
        QPen p;
        p.setColor(Qt::green);
        p.setWidth(2);
        painter->setPen(p);
        QRectF radarRect(-r,-r,r*2,r*2);
        //painter->drawEllipse(radarRect);

        {
            QMutexLocker lock(&m_radar_image_mutex);
            painter->drawImage(radarRect, m_radar_image);
        }

    }
    
}

void RadarDisplay::sectorAdded()
{
    //prepareGeometryChange();
    update();
}

void RadarDisplay::radarCallback(const marine_sensor_msgs::RadarSector::ConstPtr &message)
{
  ROS_DEBUG_STREAM("now: " << ros::Time::now() << " Radar timestamp: " << message->header.stamp);
  if (m_show_radar && !message->intensities.empty())
  {
    double angle1 = message->angle_min;
    double angle2 = message->angle_max;
    double range = message->range_max;
    int w = message->intensities.front().echoes.size();
    int h = message->intensities.size();
    QImage * sector = new QImage(w,h,QImage::Format_Grayscale8);
    sector->fill(Qt::darkGray);
    for(int i = 0; i < h; i++)
      for(int j = 0; j < w; j++)
        sector->bits()[(h-1-i)*w+j] = message->intensities[i].echoes[j]*255; // convert from float to 8 bits

    Sector s;
    if(angle1 < angle2)
        s.angle1 = angle1 + (2*M_PI);
    else
        s.angle1 = angle1;
    s.angle2 = angle2;
    s.half_scanline_angle = (s.angle1 - s.angle2)/(2.0*sector->height());
    s.range = range;
    s.sectorImage = sector;
    s.timestamp = message->header.stamp;
    s.frame_id = message->header.frame_id.c_str();
    //ROS_INFO_STREAM("angles: " << s.angle1 << " - " << s.angle2 << " range: " << range << " half angle: " << s.half_scanline_angle);
    QMutexLocker lock(&m_new_sectors_mutex);
    m_new_sectors.push_back(s);
    m_radar_frame = s.frame_id.toStdString();
  }
}


void RadarDisplay::updateRadarImage()
{
  while(true)
  {
    if ( QThread::currentThread()->isInterruptionRequested() )
      return;

    if(!m_opengl_initialized)
      initializeGL();

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

    ros::Time now = ros::Time::now();
    float persistance = 3.0;
    {
      QMutexLocker lock(&m_new_sectors_mutex);
      while(!m_new_sectors.empty())
      {
        m_sectors.push_back(m_new_sectors.front());
        m_new_sectors.pop_front();
      }
    }
    while(!m_sectors.empty() && m_sectors.front().timestamp + ros::Duration(persistance) < now)
    {
      if(m_sectors.front().sectorImage)
        delete m_sectors.front().sectorImage;
      if(m_sectors.front().sectorTexture)
        delete m_sectors.front().sectorTexture;
      m_sectors.pop_front();
    }
    {
      QMutexLocker range_lock(&m_range_mutex);
      if(m_sectors.empty())
        m_range = 0.0;
      else
        m_range = m_sectors.back().range;
    }
    for(Sector &s: m_sectors)
    {
      if(s.sectorImage && !s.rendered)
      {
        float fade = 1.0-((now-s.timestamp).toSec()/persistance);
        if(!s.sectorTexture)
        {
          s.sectorTexture = new QOpenGLTexture(*s.sectorImage);
        }
        if(!s.have_yaw)
        {
          // std::cerr << "tf buffer? " << m_tf_buffer << std::endl;
          // std::cerr << "map frame: " << m_mapFrame << " timestamp: " << s.timestamp << " frame_id: " << s.frame_id.toStdString();
          // if(m_tf_buffer && !m_mapFrame.empty())
          //    std::cerr << " can transform? " << m_tf_buffer->canTransform(m_mapFrame, s.frame_id.toStdString(), s.timestamp, ros::Duration(1.0)) << std::endl;
          // std::cerr << std::endl;
          if(m_mapFrame.empty())
          {
            if(m_tf_buffer)
            {
              std::map<std::string, std::string> parents;
              auto frames = YAML::Load(m_tf_buffer->allFramesAsYAML());
              for(auto f: frames)
                parents[f.first.as<std::string>()] = f.second["parent"].as<std::string>();
              for(auto p:parents)
                ROS_INFO_STREAM(p.first << " child of " << p.second);
              auto cursor = parents.find(s.frame_id.toStdString());
              while(cursor != parents.end())
              {
                QString parent = cursor->second.c_str();
                if(parent.endsWith("/map"))
                {
                  m_mapFrame = cursor->second;
                  break;
                }
                cursor = parents.find(cursor->second);
              }
            }
          }
          ros::Time now = ros::Time::now();
          if(m_tf_buffer && !m_mapFrame.empty())
              try
              {
                  geometry_msgs::TransformStamped t = m_tf_buffer->lookupTransform(m_mapFrame, s.frame_id.toStdString(), s.timestamp, ros::Duration(1.0));
                  double yaw = tf2::getYaw(t.transform.rotation);
                  while (yaw < 0.0)
                      yaw += (2.0*M_PI);
                  s.yaw = yaw;
                  s.angle1 = std::fmod(s.angle1+yaw,M_PI*2);
                  if(s.angle1 < 0)
                    s.angle1 += M_PI*2;
                  s.angle2 = std::fmod(s.angle2+yaw,M_PI*2);
                  if(s.angle2 < 0)
                    s.angle2 += M_PI*2;

                  s.have_yaw = true;
                  //std::cerr << "radar sector yaw: " << s.yaw << " map frame: " << m_mapFrame << std::endl;

              }
              catch (tf2::TransformException &ex)
              {
                ROS_WARN_STREAM_THROTTLE(2.0,"Unable to find transform to generate display: " << ex.what() << " lookup call time: " << now << " now: " << ros::Time::now());
              }
        }
        if(s.have_yaw)
        {
          m_program->setUniformValue("minAngle", GLfloat(s.angle2-s.half_scanline_angle*1.1));
          m_program->setUniformValue("maxAngle", GLfloat(s.angle1+s.half_scanline_angle*1.1));
          m_program->setUniformValue("fade", GLfloat(fade));
          m_program->setUniformValue("color", m_color);
          
          s.sectorTexture->bind();
          glDisable(GL_BLEND);
          glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
          //s.rendered = true;
        }
      }
    }

    {
      QMutexLocker lock(&m_radar_image_mutex);
      m_radar_image = m_fbo->toImage();
    }
    update();
    QThread::currentThread()->msleep(100);
  }

} 

void RadarDisplay::showRadar(bool show)
{
    m_show_radar = show;
    update();
}

const QColor& RadarDisplay::getColor() const
{
    return m_color;
}

void RadarDisplay::setColor(QColor color)
{
    QMutexLocker lock(&m_color_mutex);
    m_color = color;
}

void RadarDisplay::updatePosition()
{
  if(!m_tf_buffer)
    return;

  std::string radar_frame;
  {
    QMutexLocker lock(&m_new_sectors_mutex);
    radar_frame = m_radar_frame;
  }

  try
  {
    geometry_msgs::TransformStamped radar_to_earth = m_tf_buffer->lookupTransform("earth", radar_frame, ros::Time());
    gz4d::GeoPointECEF ecef_point;
    ecef_point[0] = radar_to_earth.transform.translation.x;
    ecef_point[1] = radar_to_earth.transform.translation.y;
    ecef_point[2] = radar_to_earth.transform.translation.z;
    gz4d::GeoPointLatLongDegrees ll = ecef_point;
    QGeoCoordinate location(ll.latitude(), ll.longitude(), ll.altitude());
    auto bg = findParentBackgroundRaster();
    if(bg)
    {
      //prepareGeometryChange();
      setPos(geoToPixel(location, bg));
      update();
    }
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0,"Unable to find transform to position display: " << ex.what());
  }

}
