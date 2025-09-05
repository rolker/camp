#include "grid_map.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include "../../map_view/web_mercator.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../node_manager.h"
#include "project11/gz4d_geo.h"
#include <tf2/utils.h>
#include "grid_layer.h"

namespace camp
{
namespace ros
{
namespace grids
{

GridMap::GridMap(MapItem* parent, NodeManager* node_manager, QString topic):
  Layer(parent, node_manager, topic), topic_(topic.toStdString())
{
  qRegisterMetaType<GridMapLayerData>("GridMapLayerData");

  connect(this, &GridMap::newLayerData, this, &GridMap::updateGridLayer);

  rclcpp::QoS qos(1);
  qos.durability_best_available();

  subscription_ = node_manager->node()->create_subscription<grid_map_msgs::msg::GridMap>(topic_, qos, std::bind(&GridMap::gridMapCallback, this, std::placeholders::_1));
  setStatus("[grid_map_msgs/msg/GridMap]");
}


void GridMap::gridMapCallback(const grid_map_msgs::msg::GridMap &data)
{
  if(!process_future_.isRunning())
  {
    process_future_ = QtConcurrent::run(this, &GridMap::processGridMap, data);
  }
}

void GridMap::processGridMap(const grid_map_msgs::msg::GridMap &data)
{
  grid_map::GridMap grid_map;
  auto node = node_manager_->node();
  rclcpp::Clock clock;
  if(!grid_map::GridMapRosConverter::fromMessage(data, grid_map))
  {
    RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), clock, 2, "Unable to convert GridMap message");
    return;
  }
  if(grid_map.getLayers().empty())
  {
    RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), clock, 2.0, "Got GridMap message with no layers");
    return;
  }
  for(const auto & layer: grid_map.getLayers())
  {
    GridMapLayerData grid_data;
    grid_data.layer_name = layer;
    auto size = grid_map.getSize();
    grid_data.grid_image = QImage(size.x(), size.y(), QImage::Format_ARGB32);
    grid_data.grid_image.fill(Qt::transparent);
    grid_data.meters_per_pixel = data.info.resolution;

    double min_value = std::numeric_limits<double>::max();
    double max_value = std::numeric_limits<double>::lowest();

    for(grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      double value = grid_map.at(layer, *iterator);
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }

    grid_data.range = std::make_pair(min_value, max_value);

    // If all values are identical, slightly adjust the range
    // This avoids division by zero and treats the value as a flag
    // Indicating a valid cell (This assumes invalid cells hold nan)
    if(min_value == max_value)
    {
      min_value -= 1.0;
    }

    if(min_value < max_value)
    {
      for(grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
      {
        double value = grid_map.at(layer, *iterator);
        if(!std::isnan(value))
        {
          value = (value - min_value) / (max_value - min_value);
          uint8_t ival = std::min(1.0,std::max(0.0, value))*255;
          grid_data.grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), QColor(ival, ival, ival, 255));
        }
      }
    }

    try
    {
      grid_data.center = transformToWebMercator(data.info.pose, data.header);
      emit newLayerData(grid_data);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN_STREAM(node->get_logger(), ex.what());
    }
  }
}

GridLayer * GridMap::gridLayer(const QString & layer_name) const
{
  for(auto item: childItems())
  {
    GridLayer * layer = qgraphicsitem_cast<GridLayer*>(item);
    if(layer && layer->objectName() == layer_name)
      return layer;
  }
  return nullptr;
}

void GridMap::updateGridLayer(const GridMapLayerData& data)
{
  GridLayer * layer = gridLayer(QString::fromStdString(data.layer_name));
  if(!layer)
  {
    layer = new GridLayer(this, node_manager_, data.layer_name.c_str());
  }
  layer->updateGridLayer(data);
}

}  // namespace grids
}  // namespace ros
}  // namespace camp
