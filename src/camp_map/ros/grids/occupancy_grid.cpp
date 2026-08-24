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
  // [camp#208] Seed the visibility mirror from real state. A QGraphicsItem is
  // VISIBLE from construction, and setVisibleHelper() returns before
  // itemChange() when the state is unchanged — so map::Layer::readSettings()'s
  // setVisible(true) fires no event. Without this seed the mirror stays false
  // for the whole life of any layer the operator never toggles, and the layer
  // renders nothing while looking enabled. Parenting is complete here, so
  // isVisible() is meaningful; this runs on the GUI thread.
  visible_.store(isVisible(), std::memory_order_relaxed);
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

/// [camp#208] The same palette as a 256-entry ARGB table, for Format_Indexed8.
/// One byte per cell instead of four: an occupancy grid IS single-byte data, so
/// rendering it 32-bit made the image four times larger than the grid it draws.
const QVector<QRgb>& costmapColorTable()
{
  static const QVector<QRgb> table = []
  {
    const auto& colors = costmapPalette();
    QVector<QRgb> rgb(256);
    for(int i = 0; i < 256; i++)
      rgb[i] = colors[i].rgba();
    return rgb;
  }();
  return table;
}

}  // namespace

QVariant OccupancyGrid::itemChange(GraphicsItemChange change, const QVariant& value)
{
  // [camp#208] GUI thread: mirror visibility for the ROS callback to read.
  if(change == ItemVisibleHasChanged)
    visible_.store(value.toBool(), std::memory_order_relaxed);
  return Layer::itemChange(change, value);
}

OccupancyGrid::~OccupancyGrid()
{
  // [camp#209] Wait for the in-flight render before this object goes away — the
  // worker is bound to `this` and writes into it. Mirrors ~GridMap() and
  // ~RasterLayer(); its absence here segfaulted camp when the layer was removed
  // from the Layers tab mid-render. The render window is wide (a 5000x5000 grid
  // is 25 million cells), so this is easy to hit, not a narrow race.
  // [camp#209] Drop the subscription FIRST: otherwise an executor-thread callback
  // can still fire while we wait and enqueue a fresh render into the object being
  // destroyed.
  shutdown_.store(true, std::memory_order_relaxed);
  subscription_.reset();
  process_future_.waitForFinished();
}

void OccupancyGrid::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &grid)
{
  // [camp#208] Do no work for a layer the operator has switched off. Rendering
  // costs a full-grid pass plus a grid-sized image and pixmap that stay resident,
  // so an unchecked costmap was one of the largest consumers in the process — work
  // nobody asked to see. Costmaps republish on their own timer, so the display
  // refreshes on the next publish after the layer is re-enabled.
  // [camp#208] Read the mirrored flag, NOT isVisible(): this runs on the ROS
  // executor thread and QGraphicsItem state belongs to the GUI thread.
  if(shutdown_.load(std::memory_order_relaxed))
    return;
  if(!visible_.load(std::memory_order_relaxed))
    return;

  if(!process_future_.isRunning())
  {
    process_future_ = QtConcurrent::run(this, &OccupancyGrid::processOccupancyGrid, grid);
  }
}

void OccupancyGrid::processOccupancyGrid(const nav_msgs::msg::OccupancyGrid &grid)
{
  OccupancyGridData data;
  // [camp#208] Indexed8, not ARGB32: a quarter of the memory for identical output,
  // since the source is one signed byte per cell and the palette has 256 entries.
  data.grid_image = QImage(grid.info.width, grid.info.height, QImage::Format_Indexed8);
  if(data.grid_image.isNull())
  {
    RCLCPP_WARN_STREAM(node_->node()->get_logger(),
                       "Failed to allocate " << grid.info.width << "x" << grid.info.height
                       << " occupancy grid image");
    return;
  }
  data.grid_image.setColorTable(costmapColorTable());

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

  // [camp#208] Write whole rows through scanLine() rather than setPixelColor() per
  // cell. The old form cost a virtual call plus a QColor conversion for every cell —
  // 25 million of them on a 5000x5000 costmap, on every update. The palette index IS
  // the pixel value in Indexed8, so the inner loop is a byte store.
  //
  // Occupancy values are 0..100 percent, -1 for unknown, and anything else illegal;
  // the table covers the whole int8 domain, so no value can index out of range.
  for(uint32_t row = 0; row < grid.info.height; row++)
  {
    const std::size_t row_start = std::size_t(row) * grid.info.width;
    // Source row 0 is the grid's south edge; the image is north-up.
    uchar* dest = data.grid_image.scanLine(int(grid.info.height - 1 - row));
    for(uint32_t col = 0; col < grid.info.width; col++)
      dest[col] = static_cast<uchar>(grid.data[row_start + col] + 128);
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
