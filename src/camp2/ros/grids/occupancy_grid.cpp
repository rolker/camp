#include "occupancy_grid.h"
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

  for(uint32_t row = 0; row < grid.info.height; row++)
  {
    // occupancy grid values are 0 to 100 percent or -1 for unknown
    auto row_start = row*grid.info.width;
    for(uint32_t col = 0; col < grid.info.width; col++)
    {
      auto value = grid.data[row_start+col];
      QColor color;
      if ( value == -1)
        color = QColor(128, 128, 128, 128);
      else if( value == 100)
        color = QColor(255, 0, 255, 225);
      else if( value == 99)
        color = QColor(0, 255, 255, 225);
      else
        color = QColor(255*value/100.0, 0, 255*(100-value)/100.0, value+100);
      data.grid_image.setPixelColor(QPoint(col, grid.info.height-1-row), color);
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

  auto map_distortion = web_mercator::metersPerUnit(data.origin);
  double scale = data.meters_per_pixel/map_distortion;

  pixmap->setTransform(QTransform::fromScale(scale, -scale));
  QPointF position(data.origin.x(), data.origin.y() + scale * data.grid_image.size().height());
  pixmap->setPos(position);
}

}  // namespace grids
}  // namespace ros
}  // namespace camp
