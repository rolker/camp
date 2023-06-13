#include "grid_map.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include "../../map_view/web_mercator.h"
#include <geometry_msgs/PoseStamped.h>
#include "../node_manager.h"
#include "gz4d_geo.h"
#include <tf2/utils.h>

namespace camp_ros
{

GridMap::GridMap(MapItem* parent, NodeManager* node_manager, QString topic):
  Layer(parent, node_manager, topic), topic_(topic.toStdString())
{
  qRegisterMetaType<GridMapLayerData>("GridMapLayerData");

  connect(this, &GridMap::newLayerData, this, &GridMap::updateGridLayer);

  subscriber_ = ros::NodeHandle().subscribe(topic_, 10, &GridMap::gridMapCallback, this);
  setStatus("[grid_map_msgs/GridMap]");
}

void GridMap::updateGridLayer(const GridMapLayerData& data)
{
  QGraphicsPixmapItem* pixmap = nullptr;
  for(auto child: childItems())
  {
    pixmap = qgraphicsitem_cast<QGraphicsPixmapItem*>(child);
    if(pixmap)
      break;
  }
  if(!pixmap)
    pixmap = new QGraphicsPixmapItem(this);

  QPixmap pm;
  pm.convertFromImage(data.grid_image);
  pixmap->setPixmap(pm);

  auto map_distortion = web_mercator::metersPerUnit(data.center);
  double scale = data.meters_per_pixel/map_distortion;

  pixmap->setTransform(QTransform::fromScale(scale, -scale));
  QPointF position(data.center.x() - scale * data.grid_image.size().width()/2.0, data.center.y() + scale * data.grid_image.size().height()/2.0);
  pixmap->setPos(position);

}

void GridMap::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr &data)
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
  std::string layer;
  for(auto l: grid_map.getLayers())
    if(l == "speed")
    {
      layer = l;
      break;
    }
  if(layer.empty())
   layer = grid_map.getLayers().front(); 
  GridMapLayerData grid_data;
  auto size = grid_map.getSize();
  grid_data.grid_image = QImage(size.x(), size.y(), QImage::Format_ARGB32);
  grid_data.meters_per_pixel = data->info.resolution;

  if(layer == "speed")
    for(grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      double value = grid_map.at(layer, *iterator);
      if(value < 0.0)
        grid_data.grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), QColor(255, 0, 0, 255));
      else
      {
        uint8_t ival = std::min(1.0,std::max(0.0, value/3.0))*255;
        grid_data.grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), QColor(255-ival, 255, 0, 255));
      }
    }
  else
    for(grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      double value = grid_map.at(layer, *iterator);
      uint8_t ival = std::min(1.0,std::max(0.0, value))*255;
      grid_data.grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), QColor(0, ival, 0, ival));
    }

  try
  {
    grid_data.center = transformToWebMercator(data->info.pose, data->info.header);
    emit newLayerData(grid_data);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Unable to find transform to earth for grid_map " << topic_ << " at lookup time: "<< data->info.header.stamp << " now: " << ros::Time::now() << " source frame: " << data->info.header.frame_id << " what: " << ex.what());
  }

  
}


} // namepsace camp_ros
