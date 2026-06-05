#include "grid_map.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include "../../map_view/web_mercator.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../node.h"
#include "marine_autonomy/gz4d_geo.h"
#include <tf2/utils.h>
#include "grid_layer.h"
#include <QMenu>
#include <QAction>
#include <QSettings>

namespace camp
{
namespace ros
{
namespace grids
{

GridMap::GridMap(MapItem* parent, Node* node, QString topic):
  Layer(parent, node, topic), topic_(topic.toStdString())
{
  qRegisterMetaType<GridMapLayerData>("GridMapLayerData");
  qRegisterMetaType<GridMapData>("GridMapData");

  connect(this, &GridMap::newGridData, this, &GridMap::updateGrid);

  rclcpp::QoS qos(1);
  qos.durability_best_available();

  subscription_ = node->node()->create_subscription<grid_map_msgs::msg::GridMap>(topic_, qos, std::bind(&GridMap::gridMapCallback, this, std::placeholders::_1));
  setStatus("[grid_map_msgs/msg/GridMap]");
}


void GridMap::gridMapCallback(const grid_map_msgs::msg::GridMap &data)
{
  last_msg_ = data;       // [camp#63] keep the latest for colormap re-render
  has_last_msg_ = true;
  if(!process_future_.isRunning())
  {
    process_future_ = QtConcurrent::run(this, &GridMap::processGridMap, data);
  }
}

void GridMap::processGridMap(const grid_map_msgs::msg::GridMap &data)
{
  grid_map::GridMap grid_map;
  auto node = node_->node();
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
  GridMapData grid_data;
  try
  {
    grid_data.center = transformToWebMercator(data.info.pose, data.header);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), ex.what());
    return;
  }
  grid_data.meters_per_pixel = data.info.resolution;

  // [camp#63] Snapshot the ramp once so this worker uses a consistent colormap
  // even if the UI thread changes it mid-render.
  const map::ColorMap cm = colormap_;

  for(const auto & layer: grid_map.getLayers())
  {
    GridMapLayerData grid_layer_data;
    grid_layer_data.layer_name = layer;
    auto size = grid_map.getSize();
    grid_layer_data.grid_image = QImage(size.x(), size.y(), QImage::Format_ARGB32);
    grid_layer_data.grid_image.fill(Qt::transparent);

    double min_value = std::numeric_limits<double>::max();
    double max_value = std::numeric_limits<double>::lowest();

    for(grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      double value = grid_map.at(layer, *iterator);
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }

    grid_layer_data.range = std::make_pair(min_value, max_value);

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
          // [camp#63] Normalised value -> colour via the selectable ramp
          // (colorNormalized clamps to [0,1]); default grayscale reproduces the
          // prior output.
          value = (value - min_value) / (max_value - min_value);
          grid_layer_data.grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), cm.colorNormalized(value));
        }
      }
    }
    grid_data.layers.push_back(grid_layer_data);
  }
  emit newGridData(grid_data);

}

void GridMap::updateGrid(const GridMapData& data)
{
  setWebMercatorPositionAndScale(data.center, data.meters_per_pixel);
  for(const auto& layer_data: data.layers)
  {
    updateGridLayer(layer_data);
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
    layer = new GridLayer(this, node_, data.layer_name.c_str());
  }
  layer->updateGridLayer(data);
}

void GridMap::setColormap(map::ColorMap::Type type)
{
  if(type == colormap_.type())
    return;
  colormap_.setType(type);
  writeSettings();
  if(has_last_msg_)               // re-render the cached grid with the new ramp
    gridMapCallback(last_msg_);
}

void GridMap::contextMenu(QMenu* menu)
{
  Layer::contextMenu(menu);
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(auto type : map::ColorMap::allTypes())
  {
    QAction* action = colormap_menu->addAction(map::ColorMap::name(type));
    action->setCheckable(true);
    action->setChecked(type == colormap_.type());
    connect(action, &QAction::triggered, this, [this, type]() { setColormap(type); });
  }
}

void GridMap::readSettings()
{
  Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  colormap_.setType(map::ColorMap::typeFromName(
    settings.value("colormap", map::ColorMap::name(colormap_.type())).toString()));
  settings.endGroup();
  settings.endGroup();
}

void GridMap::writeSettings()
{
  Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  settings.setValue("colormap", map::ColorMap::name(colormap_.type()));
  settings.endGroup();
  settings.endGroup();
}

}  // namespace grids
}  // namespace ros
}  // namespace camp
