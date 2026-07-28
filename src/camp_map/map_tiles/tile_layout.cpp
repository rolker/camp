#include "tile_layout.h"
#include "tile_address.h"

#include <iomanip>
#include <sstream>

namespace camp
{

namespace map_tiles
{

QRectF TileLayout::boundingRect() const
{
  if(!zoom_levels.empty())
    return zoom_levels.front().boundingRect();
  return {};
}

QRectF TileLayout::ZoomLevel::boundingRect() const
{
  return QRectF(top_left_corner, QSizeF(scale*tile_width*matrix_width, scale*tile_height*matrix_height));
}

std::string TileLayout::getUrl(const TileAddress& address) const
{
  std::string url;
  for(std::size_t i = 0; i < std::max(url_static_parts.size(), url_variable_keys.size()); i++)
  {
    if(i < url_static_parts.size())
      url += url_static_parts[i];
    if(i < url_variable_keys.size())
    {
      auto key = url_variable_keys[i];
      if(key == "TileMatrix")
        url += std::to_string(address.zoomLevel());
      if(key == "TileRow")
        url += std::to_string(address.index().y());
      if(key == "TileCol")
        url += std::to_string(address.index().x());
      // [camp#118] WMS per-tile GetMap: expand to this tile's EPSG:3857 bbox
      // (minx,miny,maxx,maxy — WMS 1.3.0 axis order for EPSG:3857 is easting,
      // northing). Two decimal places (centimeter precision in Web-Mercator
      // meters) keeps URLs stable and short.
      if(key == "WMS_BBOX")
      {
        const auto& level = zoom_levels[address.zoomLevel()];
        const QPointF top_left = address.topLeftCorner();
        const double min_x = top_left.x();
        const double max_y = top_left.y();
        const double max_x = min_x + level.scale*level.tile_width;
        const double min_y = max_y - level.scale*level.tile_height;
        std::ostringstream bbox;
        bbox << std::fixed << std::setprecision(2)
             << min_x << ',' << min_y << ',' << max_x << ',' << max_y;
        url += bbox.str();
      }
    }
  }
  return url;
}


} // namespace map_tiles

} // namespace camp
