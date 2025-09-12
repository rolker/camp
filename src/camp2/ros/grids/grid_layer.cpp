#include "grid_layer.h"
#include "grid_map.h"
#include "../../map_view/web_mercator.h"

namespace camp
{
namespace ros
{
namespace grids
{

GridLayer::GridLayer(MapItem* parent, Node* node, QString layer_name):
  Layer(parent, node, layer_name)
{
}

void GridLayer::updateGridLayer(const GridMapLayerData& data)
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

  std::stringstream status;
  status << "Range: (" << data.range.first << ", " << data.range.second << ")";

  setStatus(status.str().c_str());
}

} // namespace grids
} // namespace ros
} // namespace camp
