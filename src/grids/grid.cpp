#include "grid.h"
#include <QPainter>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include "gz4d_geo.h"
#include <QDebug>
#include <geometry_msgs/PoseStamped.h>
#include "backgroundraster.h"
#include <grid_map_ros/grid_map_ros.hpp>

Grid::Grid(QWidget* parent, QGraphicsItem *parentItem):
  QWidget(parent),
  GeoGraphicsItem(parentItem)
{
  ui_.setupUi(this);
  connect(this, &Grid::newGridMadeAvaiable, this, &Grid::newGridAvailable, Qt::QueuedConnection);
  connect(ui_.displayCheckBox, &QCheckBox::stateChanged, this, &Grid::visibilityChanged);
}

QRectF Grid::boundingRect() const
{
  if(current_grid_ && is_visible_)
  {
    auto size = current_grid_->grid_image.size();
    auto scale = current_grid_->meters_per_pixel/pixel_size_;
    QRectF ret(-scale*size.width()/2.0,-scale*size.height()/2.0, size.width()*scale, size.height()*scale);
    return ret;
  }
  return QRectF();
}

void Grid::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  if(current_grid_ && is_visible_)
  {
    painter->save();
    auto scale = current_grid_->meters_per_pixel/pixel_size_;
    painter->scale(scale, scale);
    auto size = current_grid_->grid_image.size();
    painter->drawImage(QPoint(-size.width()/2.0, -size.height()/2.0), current_grid_->grid_image);
    painter->restore();
  }
}

void Grid::setTopic(std::string topic, std::string type)
{
  if(!spinner_)
  {
    spinner_ = std::make_shared<ros::AsyncSpinner>(1, &ros_queue_);
    spinner_->start();
  }
  ros::SubscribeOptions ops;
  
  if(type == "nav_msgs/OccupancyGrid")
    ops = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>(topic, 1, boost::bind(&Grid::occupancyGridCallback, this, _1), ros::VoidPtr(), &ros_queue_);
  if(type == "grid_map_msgs/GridMap")
    ops = ros::SubscribeOptions::create<grid_map_msgs::GridMap>(topic, 1, boost::bind(&Grid::gridMapCallback, this, _1), ros::VoidPtr(), &ros_queue_);
  subscriber_ = ros::NodeHandle().subscribe(ops);

  ui_.topicLabel->setText(topic.c_str());
}

void Grid::setTF2Buffer(tf2_ros::Buffer* buffer)
{
  tf_buffer_ = buffer;
}

void Grid::setPixelSize(double s)
{
  pixel_size_ = s;
}

void Grid::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &data)
{
  auto grid_data = std::make_shared<GridData>();
  grid_data->grid_image = QImage(data->info.width, data->info.height, QImage::Format_ARGB32);
  grid_data->meters_per_pixel = data->info.resolution;
  for(int row = 0; row < data->info.height; row++)
  {
    // occupancy grid values are 0 to 100 percent or -1 for unknown
    auto row_start = row*data->info.width;
    for(int col = 0; col < data->info.width; col++)
    {
      QColor color;
      if( data->data[row_start+col] < 0)
        color = QColor(128, 128, 128, 128);
      else if( data->data[row_start+col] >= 100)
        color = QColor(255, 255, 255, 255);
      else
        color = QColor(0, 255, 0, data->data[row_start+col]*2.55); // 0-100 -> 0-255
      grid_data->grid_image.setPixelColor(QPoint(col, data->info.height-1-row), color);
    }
  }
  geometry_msgs::Pose center_pose = data->info.origin;
  center_pose.position.x += data->info.resolution*data->info.width/2.0;
  center_pose.position.y += data->info.resolution*data->info.height/2.0;
  grid_data->center = getGeoCoordinate(center_pose, data->header);
  {
    std::lock_guard<std::mutex> lock(new_grid_mutex_);
    new_grid_ = grid_data;
  }
  emit newGridMadeAvaiable();
}

void Grid::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &data)
{
  grid_map::GridMap grid_map;
  if(!grid_map::GridMapRosConverter::fromMessage(*data, grid_map))
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Unable to convert GridMap message");
    return;
  }
  if(grid_map.getLayers().empty())
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Got GridMap message with no layers");
    return;
  }
  std::string layer = grid_map.getLayers().front(); 
  auto grid_data = std::make_shared<GridData>();
  auto size = grid_map.getSize();
  grid_data->grid_image = QImage(size.x(), size.y(), QImage::Format_ARGB32);
  grid_data->meters_per_pixel = data->info.resolution;

  for(grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
  {
    double value = grid_map.at(layer, *iterator);
    uint8_t ival = std::min(1.0,std::max(0.0, value))*255;
    grid_data->grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), QColor(0, ival, 0, ival));
  }

  grid_data->center = getGeoCoordinate(data->info.pose, data->info.header);
  {
    std::lock_guard<std::mutex> lock(new_grid_mutex_);
    new_grid_ = grid_data;
  }
  emit newGridMadeAvaiable();
}

QGeoCoordinate Grid::getGeoCoordinate(const geometry_msgs::Pose &pose, const std_msgs::Header &header)
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


void Grid::newGridAvailable()
{
  if(current_grid_)
    prepareGeometryChange();

  {
    std::lock_guard<std::mutex> lock(new_grid_mutex_);
    current_grid_ = new_grid_;
    new_grid_.reset();
  }

  auto bg = findParentBackgroundRaster();
  if(bg && current_grid_)
  {
    setPos(geoToPixel(current_grid_->center, bg));
  }

  GeoGraphicsItem::update();
}

void Grid::visibilityChanged()
{
  prepareGeometryChange();
  is_visible_ = ui_.displayCheckBox->isChecked();
  GeoGraphicsItem::update();
}


void Grid::updateBackground(BackgroundRaster * bg)
{
  prepareGeometryChange();
  setParentItem(bg);
  if(bg)
  {
    setPixelSize(bg->pixelSize());
    if(current_grid_)
      setPos(geoToPixel(current_grid_->center, bg));
  }
  GeoGraphicsItem::update();
}