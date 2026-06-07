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


GridMap::~GridMap()
{
  // Stop the worker from relaunching, then wait for the in-flight render so it
  // can't touch this object after destruction.
  QFuture<void> pending;
  {
    QMutexLocker lock(&mutex_);
    shutdown_ = true;
    pending = process_future_;
  }
  pending.waitForFinished();
}

void GridMap::startRenderLocked()
{
  // Snapshot the inputs under the lock so the worker is fully isolated from
  // mutex_-guarded state.
  rendering_ = true;
  process_future_ = QtConcurrent::run(this, &GridMap::processGridMap, last_msg_, colormap_);
}

void GridMap::requestRenderLocked()
{
  if(rendering_)
    render_pending_ = true;   // coalesce; onProcessFinished() will pick it up
  else
    startRenderLocked();
}

void GridMap::onProcessFinished()
{
  QMutexLocker lock(&mutex_);
  if(!shutdown_ && render_pending_)
  {
    render_pending_ = false;
    startRenderLocked();      // rendering_ stays true across the relaunch
  }
  else
    rendering_ = false;
}

void GridMap::gridMapCallback(const grid_map_msgs::msg::GridMap &data)
{
  QMutexLocker lock(&mutex_);
  last_msg_ = data;       // [camp#63] keep the latest for colormap re-render
  has_last_msg_ = true;
  requestRenderLocked();
}

void GridMap::processGridMap(grid_map_msgs::msg::GridMap data, map::ColorMap colormap)
{
  GridMapData grid_data;
  if(renderToData(data, colormap, grid_data))
    emit newGridData(grid_data);
  onProcessFinished();
}

bool GridMap::renderToData(const grid_map_msgs::msg::GridMap &data,
                           const map::ColorMap &colormap, GridMapData &grid_data)
{
  grid_map::GridMap grid_map;
  auto node = node_->node();
  rclcpp::Clock clock;
  if(!grid_map::GridMapRosConverter::fromMessage(data, grid_map))
  {
    RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), clock, 2000, "Unable to convert GridMap message");
    return false;
  }
  if(grid_map.getLayers().empty())
  {
    RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), clock, 2000, "Got GridMap message with no layers");
    return false;
  }
  try
  {
    grid_data.center = transformToWebMercator(data.info.pose, data.header);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN_STREAM(node->get_logger(), ex.what());
    return false;
  }
  grid_data.meters_per_pixel = data.info.resolution;

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
          grid_layer_data.grid_image.setPixelColor(QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()), colormap.colorNormalized(value));
        }
      }
    }
    grid_data.layers.push_back(grid_layer_data);
  }
  return true;
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
  {
    QMutexLocker lock(&mutex_);
    if(type == colormap_.type())
      return;
    colormap_.setType(type);
    if(has_last_msg_)             // re-render the cached grid with the new ramp
      requestRenderLocked();      // coalesced: not lost even if a render is in flight
  }
  writeSettings();                // QSettings I/O outside the lock
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
  auto type = map::ColorMap::typeFromName(
    settings.value("colormap", map::ColorMap::name(colormap_.type())).toString());
  settings.endGroup();
  settings.endGroup();
  QMutexLocker lock(&mutex_);
  colormap_.setType(type);
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
