#include "occupancy_grid.h"

#include <array>

#include <marine_colormap/occupancy.hpp>

#include "../../map_view/web_mercator.h"
#include "../node.h"

namespace camp
{
namespace ros
{
namespace grids
{

OccupancyGrid::OccupancyGrid(MapItem* parent, Node* node, QString topic)
  : Layer(parent, node, topic)
{
  qRegisterMetaType<OccupancyGridData>("OccupancyGridData");
  qRegisterMetaType<nav_msgs::msg::OccupancyGrid>("nav_msgs::msg::OccupancyGrid");

  connect(this, &OccupancyGrid::occupancyGridUpdated, this, &OccupancyGrid::updateOccupancyGrid);

  rclcpp::QoS qos(1);
  qos.durability_best_available();

  // Initialize the subscription to the occupancy grid topic
  subscription_ = node->node()->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic.toStdString(), qos,
      std::bind(&OccupancyGrid::occupancyGridCallback, this, std::placeholders::_1));

  setStatus("[nav_msgs/msg/OccupancyGrid]");
}

namespace
{

/// The shared rviz-exact costmap colours (marine_colormap#19), flattened to a
/// 256-entry table indexed by `value + 128` so the per-cell cost is one array
/// read rather than a walk of the lookup table's entries. Built once.
///
/// Alpha comes straight from the palette: free space is fully transparent and
/// everything else opaque. Camp's own per-layer opacity control (the map tree's
/// "opacity:" spin box, MapItem::setOpacity) provides see-through against the
/// chart underneath — which is better than the alpha ramp this replaced, where
/// opacity varied with cost and so encoded the same quantity twice.
const std::array<QColor, 256>& costmapPalette()
{
  static const std::array<QColor, 256> palette = []
  {
    const auto table = marine_colormap::occupancy_costmap_table();
    std::array<QColor, 256> colors;
    for(int i = 0; i < 256; i++)
    {
      const auto c = table.lookup(static_cast<float>(i - 128));
      colors[i] = QColor::fromRgbF(c.r, c.g, c.b, c.a);
    }
    return colors;
  }();
  return palette;
}

}  // namespace

void OccupancyGrid::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid)
{
  if(!process_future_.isRunning())
  {
    process_future_ = QtConcurrent::run(this, &OccupancyGrid::processOccupancyGrid, grid);
  }
}

void OccupancyGrid::processOccupancyGrid(const nav_msgs::msg::OccupancyGrid &grid)
{
  OccupancyGridData data;
  data.grid_image = QImage(grid.info.width, grid.info.height, QImage::Format_ARGB32);

  try
  {
    data.origin = transformToWebMercator(grid.info.origin, grid.header);
    // todo: handle rotation
  }
  catch(const std::exception& e)
  {
    RCLCPP_WARN_STREAM(node_->node()->get_logger(), "Failed to transform occupancy grid origin: " << e.what());
    return;
  }

  data.meters_per_pixel = grid.info.resolution;

  const auto& palette = costmapPalette();

  for(uint32_t row = 0; row < grid.info.height; row++)
  {
    // occupancy grid values are 0 to 100 percent, -1 for unknown, and anything
    // else illegal; the palette covers the whole int8 domain.
    auto row_start = row*grid.info.width;
    for(uint32_t col = 0; col < grid.info.width; col++)
    {
      auto value = grid.data[row_start+col];
      data.grid_image.setPixelColor(QPoint(col, grid.info.height-1-row),
                                    palette[static_cast<uint8_t>(value + 128)]);
    }
  }

  emit occupancyGridUpdated(data);
}

void OccupancyGrid::updateOccupancyGrid(const OccupancyGridData &data)
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

  setWebMercatorPositionAndScale(data.origin, data.meters_per_pixel);
  pixmap->setTransform(QTransform::fromScale(1.0, -1.0));
  pixmap->setPos(0.0, data.grid_image.size().height());
}

}  // namespace grids
}  // namespace ros
}  // namespace camp
