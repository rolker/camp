#include "grid_map.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include "../../map_view/web_mercator.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "../node.h"
#include "marine_autonomy/gz4d_geo.h"
#include <tf2/utils.h>
#include "grid_layer.h"
#include <marine_colormap/palette.hpp>
#include <marine_colormap/color.hpp>
#include <QColor>
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
  // [camp#209] Drop the subscription FIRST. shutdown_ alone was not enough: a
  // callback arriving after it was set could still take the mutex and reach
  // requestRenderLocked(), launching a NEW worker bound to `this` that the
  // captured `pending` future below does not cover. Resetting the subscription
  // stops callbacks at the source.
  subscription_.reset();

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
  process_future_ = QtConcurrent::run(this, &GridMap::processGridMap, last_msg_, colormap_name_);
}

void GridMap::requestRenderLocked()
{
  // [camp#209] Never start work once teardown has begun — belt to the
  // subscription reset in the destructor.
  if(shutdown_)
    return;
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

QVariant GridMap::itemChange(GraphicsItemChange change, const QVariant& value)
{
  // [camp#208] Becoming visible is the only chance to draw a latched dataset that
  // arrived while this layer was hidden — nothing will republish it.
  if(change == ItemVisibleHasChanged)
  {
    const bool shown = value.toBool();
    // [camp#208] Mirror for the ROS callback thread — see visible_.
    visible_.store(shown, std::memory_order_relaxed);
    if(shown)
    {
      QMutexLocker lock(&mutex_);
      if(has_last_msg_)
        requestRenderLocked();
    }
  }
  return Layer::itemChange(change, value);
}

void GridMap::gridMapCallback(const grid_map_msgs::msg::GridMap &data)
{
  QMutexLocker lock(&mutex_);
  last_msg_ = data;       // [camp#63] keep the latest for colormap re-render
  has_last_msg_ = true;
  // [camp#208] Keep the message (a re-render on show or on a colormap change
  // needs it) but do NOT rasterise for a layer the operator has switched off.
  // The render is the expensive half: a grid-sized ARGB image plus its pixmap,
  // held for as long as the layer exists. itemChange() above repaints on show.
  // [camp#208] visible_ not isVisible(): this is the ROS callback thread.
  if(visible_.load(std::memory_order_relaxed))
    requestRenderLocked();
}

void GridMap::processGridMap(grid_map_msgs::msg::GridMap data, std::string colormap_name)
{
  GridMapData grid_data;
  if(renderToData(data, colormap_name, grid_data))
    emit newGridData(grid_data);
  onProcessFinished();
}

bool GridMap::renderToData(const grid_map_msgs::msg::GridMap &data,
                           const std::string &colormap_name, GridMapData &grid_data)
{
  // [camp#141] Resolve the palette once for the whole message; unknown -> grayscale.
  const marine_colormap::Palette* palette = marine_colormap::find_palette(colormap_name);
  if(!palette)
    palette = marine_colormap::find_palette("grayscale");
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
          // [camp#63 / camp#141] Normalised value -> colour via the selected
          // marine_colormap palette (Palette::sample clamps to [0,1]); invalid
          // (NaN) cells are left transparent by the fill above. Default grayscale.
          value = (value - min_value) / (max_value - min_value);
          const marine_colormap::Rgba8 c =
            marine_colormap::to_rgba8(palette->sample(static_cast<float>(value)));
          grid_layer_data.grid_image.setPixelColor(
            QPoint(size.x()-1-iterator.getUnwrappedIndex().x(), iterator.getUnwrappedIndex().y()),
            QColor(c.r, c.g, c.b, c.a));
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

void GridMap::setColormap(const std::string& name)
{
  {
    QMutexLocker lock(&mutex_);
    if(name == colormap_name_)
      return;
    colormap_name_ = name;
    if(has_last_msg_)             // re-render the cached grid with the new ramp
      requestRenderLocked();      // coalesced: not lost even if a render is in flight
  }
  writeSettings();                // QSettings I/O outside the lock
}

void GridMap::contextMenu(QMenu* menu)
{
  Layer::contextMenu(menu);
  // [camp#141] Expose the FULL marine_colormap registry, not just the legacy ramps.
  std::string current;
  {
    QMutexLocker lock(&mutex_);
    current = colormap_name_;
  }
  QMenu* colormap_menu = menu->addMenu("Colormap");
  for(const std::string& name : marine_colormap::palette_names())
  {
    QAction* action = colormap_menu->addAction(QString::fromStdString(name));
    action->setCheckable(true);
    action->setChecked(name == current);
    connect(action, &QAction::triggered, this, [this, name]() { setColormap(name); });
  }
}

void GridMap::readSettings()
{
  Layer::readSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  // [camp#141] Persisted palette name; case-insensitive read + registry-validated
  // (unknown -> grayscale).
  // Literal default avoids reading colormap_name_ without holding mutex_; the
  // registry-validation below already falls back to "grayscale" for unknowns.
  std::string name = settings.value(
    "colormap", "grayscale").toString().toLower().toStdString();
  if(!marine_colormap::palette_index(name))
    name = "grayscale";
  settings.endGroup();
  settings.endGroup();
  QMutexLocker lock(&mutex_);
  colormap_name_ = name;
}

void GridMap::writeSettings()
{
  Layer::writeSettings();
  QSettings settings;
  settings.beginGroup("MapItem");
  settings.beginGroup(itemID());
  std::string name;
  {
    QMutexLocker lock(&mutex_);   // snapshot the name; QSettings I/O stays unlocked
    name = colormap_name_;
  }
  settings.setValue("colormap", QString::fromStdString(name));
  settings.endGroup();
  settings.endGroup();
}

}  // namespace grids
}  // namespace ros
}  // namespace camp
