#include "grid.h"
#include <QPainter>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "project11/gz4d_geo.h"
#include <QDebug>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "backgroundraster.h"
#include <grid_map_ros/grid_map_ros.hpp>

Grid::Grid(QWidget* parent, QGraphicsItem *parentItem)
  :camp_ros::ROSWidget(parent),
  GeoGraphicsItem(parentItem)
{
  ui_.setupUi(this);
  connect(this, &Grid::newGridMadeAvailable, this, &Grid::newGridAvailable, Qt::QueuedConnection);
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
  topic_ = topic;
  type_ = type;

  ui_.topicLabel->setText(topic.c_str());
  visibilityChanged();
}

void Grid::setPixelSize(double s)
{
  pixel_size_ = s;
}

void Grid::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &data)
{
  auto grid_data = std::make_shared<GridData>();
  grid_data->grid_image = QImage(data.info.width, data.info.height, QImage::Format_ARGB32);
  grid_data->meters_per_pixel = data.info.resolution;
  for(int row = 0; row < data.info.height; row++)
  {
    // occupancy grid values are 0 to 100 percent or -1 for unknown
    auto row_start = row*data.info.width;
    for(int col = 0; col < data.info.width; col++)
    {
      QColor color;
      if( data.data[row_start+col] < 0)
        color = QColor(128, 128, 128, 128);
      else if( data.data[row_start+col] >= 100)
        color = QColor(255, 255, 255, 255);
      else
        color = QColor(0, 255, 0, data.data[row_start+col]*2.55); // 0-100 -> 0-255
      grid_data->grid_image.setPixelColor(QPoint(col, data.info.height-1-row), color);
    }
  }
  geometry_msgs::msg::Pose center_pose = data.info.origin;
  center_pose.position.x += data.info.resolution*data.info.width/2.0;
  center_pose.position.y += data.info.resolution*data.info.height/2.0;
  grid_data->center = getGeoCoordinate(center_pose, data.header);
  {
    std::lock_guard<std::mutex> lock(new_grid_mutex_);
    new_grid_ = grid_data;
  }
  emit newGridMadeAvailable();
}

void Grid::gridMapCallback(const grid_map_msgs::msg::GridMap &data)
{
  grid_map::GridMap grid_map;
  if(!grid_map::GridMapRosConverter::fromMessage(data, grid_map))
  {
    rclcpp::Clock clock;
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 2000, "Unable to convert GridMap message");
    return;
  }
  if(grid_map.getLayers().empty())
  {
    rclcpp::Clock clock;
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 2000, "Got GridMap message with no layers");
    return;
  }
  std::string layer;
  for(auto l: grid_map.getLayers())
    if(l == "speed")
    {
      layer = l;
      break;
    }
  if(layer.empty())
   layer = grid_map.getLayers().front(); 
  auto grid_data = std::make_shared<GridData>();
  auto size = grid_map.getSize();
  grid_data->grid_image = QImage(size.x(), size.y(), QImage::Format_ARGB32);
  grid_data->meters_per_pixel = data.info.resolution;

  if(layer == "speed")
    for(grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      double value = grid_map.at(layer, *iterator);
      if(value < 0.0)
        grid_data->grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), QColor(255, 0, 0, 128));
      else
      {
        uint8_t ival = std::min(1.0,std::max(0.0, value/3.0))*255;
        grid_data->grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), QColor(255-ival, 255, 0, 128));
      }
    }
  else
    for(grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      double value = grid_map.at(layer, *iterator);
      uint8_t ival = std::min(1.0,std::max(0.0, value))*255;
      grid_data->grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), QColor(0, ival, 0, ival));
    }

  grid_data->center = getGeoCoordinate(data.info.pose, data.header);
  {
    std::lock_guard<std::mutex> lock(new_grid_mutex_);
    new_grid_ = grid_data;
  }
  emit newGridMadeAvailable();
}

QGeoCoordinate Grid::getGeoCoordinate(const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header)
{
  if(header.frame_id.empty())
  {
    rclcpp::Clock clock;
    RCLCPP_DEBUG_STREAM_THROTTLE(node_->get_logger(), clock, 1000, "Grid: Missing frame_id");
    return {};
  }
  try
  {
    geometry_msgs::msg::PoseStamped grid_corner;
    grid_corner.header = header;
    grid_corner.pose = pose;
    auto ecef = transform_buffer_->transform(grid_corner, "earth", tf2::durationFromSec(0.5));

    gz4d::GeoPointECEF ecef_point;
    ecef_point[0] = ecef.pose.position.x;
    ecef_point[1] = ecef.pose.position.y;
    ecef_point[2] = ecef.pose.position.z;
    gz4d::GeoPointLatLongDegrees ll = ecef_point;
    return QGeoCoordinate(ll.latitude(), ll.longitude(), ll.altitude());
  }
  catch (tf2::TransformException &ex)
  {
    rclcpp::Clock clock;
    RCLCPP_WARN_STREAM_THROTTLE(node_->get_logger(), clock, 2000, "Unable to find transform to earth for grid: " << ex.what() << " lookup time: " << rclcpp::Time(header.stamp).seconds() << " now: " << node_->get_clock()->now().seconds() << " source frame: " << header.frame_id);
  }
  return {};
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
  if(is_visible_ && node_)
  {
    if(type_ == "nav_msgs/OccupancyGrid")
      occupancy_grid_subscription_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(topic_, 1, std::bind(&Grid::occupancyGridCallback, this, std::placeholders::_1));
    if(type_ == "grid_map_msgs/GridMap")
      grid_map_subscription_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(topic_, 1, std::bind(&Grid::gridMapCallback, this, std::placeholders::_1));
  }
  else
  {
    occupancy_grid_subscription_.reset();
    grid_map_subscription_.reset();
  }

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