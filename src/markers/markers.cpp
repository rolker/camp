#include "markers.h"
#include <QPainter>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "gz4d_geo.h"
#include <QDebug>
#include <geometry_msgs/PoseStamped.h>
#include "backgroundraster.h"
#include <grid_map_ros/grid_map_ros.hpp>

Markers::Markers(QWidget* parent, QGraphicsItem *parentItem):
  QWidget(parent),
  GeoGraphicsItem(parentItem)
{
  ui_.setupUi(this);
  connect(this, &Markers::newMarkersMadeAvailable, this, &Markers::newMarkersAvailable, Qt::QueuedConnection);
  connect(ui_.displayCheckBox, &QCheckBox::stateChanged, this, &Markers::visibilityChanged);
}

QRectF Markers::boundingRect() const
{
  QRectF ret;
  if(!current_markers_.empty() && is_visible_)
  {
    for(auto ns: current_markers_)
      for(auto m: ns.second)
      {
        switch(m.second->marker.type)
        {
          case visualization_msgs::Marker::SPHERE:
          {
            QRectF bbox(m.second->local_position, m.second->local_position);
            ret = ret|bbox.marginsAdded(QMarginsF(m.second->marker.scale.x, m.second->marker.scale.y, m.second->marker.scale.x, m.second->marker.scale.y));
            break;
          }
          case visualization_msgs::Marker::LINE_STRIP:
          {
            QRectF bbox(m.second->local_position, m.second->local_position);
            for(auto p: m.second->marker.points)
              bbox |= QRectF(QPointF(p.x, p.y),QPointF(p.x, p.y));
            ret = ret|bbox;
          }
        }
      }

  }
  return ret;
}

void Markers::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  if(!current_markers_.empty() && is_visible_)
    for(auto ns: current_markers_)
      for(auto m: ns.second)
      {
        painter->save();
        QPen p;
        p.setColor(QColor(m.second->marker.color.r*255, m.second->marker.color.g*255, m.second->marker.color.b*255, m.second->marker.color.a*255));
        painter->setPen(p);
        QBrush b = painter->brush();
        b.setColor(QColor(m.second->marker.color.r*255, m.second->marker.color.g*255, m.second->marker.color.b*255, m.second->marker.color.a*128));
        b.setStyle(Qt::BrushStyle::SolidPattern);
        
        painter->setBrush(b);
        switch(m.second->marker.type)
        {
          case visualization_msgs::Marker::SPHERE:
          {
            QRectF bbox(m.second->local_position, m.second->local_position);
            painter->drawEllipse(bbox.marginsAdded(QMarginsF(m.second->marker.scale.x, m.second->marker.scale.y, m.second->marker.scale.x, m.second->marker.scale.y)));
            break;
          }
          case visualization_msgs::Marker::LINE_STRIP:
          {
            for(auto p1 = m.second->marker.points.begin(); p1 != m.second->marker.points.end(); ++p1)
            {
              auto p2 = p1;
              p2++;
              if(p2 != m.second->marker.points.end())
              {
                QLineF line(m.second->local_position+QPointF(p1->x, p1->y), m.second->local_position+QPointF(p2->x, p2->y));
                painter->drawLine(line);
              }
            }
            break;
          }
          default:
            ROS_WARN_STREAM("marker type not handles: " << m.second->marker.type);
        }
        painter->restore();
      }
}

void Markers::setTopic(std::string topic, std::string type)
{
  if(!spinner_)
  {
    spinner_ = std::make_shared<ros::AsyncSpinner>(1, &ros_queue_);
    spinner_->start();
  }
  ros::SubscribeOptions ops;
  
  if(type == "visualization_msgs/MarkerArray")
    ops = ros::SubscribeOptions::create<visualization_msgs::MarkerArray>(topic, 1, boost::bind(&Markers::markerArrayCallback, this, _1), ros::VoidPtr(), &ros_queue_);
  if(type == "visualization_msgs/Marker")
    ops = ros::SubscribeOptions::create<visualization_msgs::Marker>(topic, 1, boost::bind(&Markers::markerCallback, this, _1), ros::VoidPtr(), &ros_queue_);
  subscriber_ = ros::NodeHandle().subscribe(ops);

  ui_.topicLabel->setText(topic.c_str());
}

void Markers::setTF2Buffer(tf2_ros::Buffer* buffer)
{
  tf_buffer_ = buffer;
}

void Markers::setPixelSize(double s)
{
  pixel_size_ = s;
}

void Markers::markerArrayCallback(const visualization_msgs::MarkerArrayConstPtr &data)
{
  addMarkers(data->markers);
}

void Markers::markerCallback(const visualization_msgs::MarkerConstPtr &data)
{
  std::vector<visualization_msgs::Marker> markers;
  markers.push_back(*data);
  addMarkers(markers);
}

void Markers::addMarkers(const std::vector<visualization_msgs::Marker> &markers)
{
  for(auto m: markers)
  {
    auto marker_data = std::make_shared<MarkerData>();
    marker_data->marker = m;
    marker_data->position = getGeoCoordinate(m.pose, m.header);
    std::lock_guard<std::mutex> lock(new_markers_mutex_);
    new_markers_.push_back(marker_data);
  }
  emit newMarkersMadeAvailable();
}

QGeoCoordinate Markers::getGeoCoordinate(const geometry_msgs::Pose &pose, const std_msgs::Header &header)
{
  QGeoCoordinate ret;
  try
  {
    geometry_msgs::PoseStamped grid_corner;
    grid_corner.header = header;
    grid_corner.pose = pose;
    auto ecef = tf_buffer_->transform(grid_corner, "earth", ros::Duration(0.5));

    gz4d::GeoPointECEF ecef_point;
    ecef_point[0] = ecef.pose.position.x;
    ecef_point[1] = ecef.pose.position.y;
    ecef_point[2] = ecef.pose.position.z;
    gz4d::GeoPointLatLongDegrees ll = ecef_point;
    ret = QGeoCoordinate(ll.latitude(), ll.longitude(), ll.altitude());
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Unable to find transform to earth for grid: " << ex.what() << " lookup time: " << header.stamp << " now: " << ros::Time::now() << " source frame: " << header.frame_id);
  }
  return ret;
}


void Markers::newMarkersAvailable()
{
  prepareGeometryChange();

  std::vector<std::shared_ptr<MarkerData> > new_markers;
  {
    std::lock_guard<std::mutex> lock(new_markers_mutex_);
    new_markers = new_markers_;
    new_markers_.clear();
  }

  auto bg = findParentBackgroundRaster();
  for(auto marker: new_markers)
  {
    switch(marker->marker.action)
    {
      case visualization_msgs::Marker::ADD:
        if(bg)
          marker->local_position = geoToPixel(marker->position, bg);
        current_markers_[marker->marker.ns][marker->marker.id] = marker;
        break;
      case visualization_msgs::Marker::DELETE:
        current_markers_[marker->marker.ns][marker->marker.id].reset();
        break;
      case visualization_msgs::Marker::DELETEALL:
        current_markers_[marker->marker.ns].clear();
        break;
      default:
        ROS_WARN_STREAM("Unknown marker action: " << marker->marker.action);
    }
  }

  GeoGraphicsItem::update();
}

void Markers::visibilityChanged()
{
  prepareGeometryChange();
  is_visible_ = ui_.displayCheckBox->isChecked();
  GeoGraphicsItem::update();
}


void Markers::updateBackground(BackgroundRaster * bg)
{
  prepareGeometryChange();
  setParentItem(bg);
  if(bg)
  {
    setPixelSize(bg->pixelSize());
  //   if(current_grid_)
  //     setPos(geoToPixel(current_grid_->center, bg));
  }
  GeoGraphicsItem::update();
}
