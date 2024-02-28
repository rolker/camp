#include "tile_layout.h"
#include "tile_address.h"

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
    }
  }
  return url;
}


} // namespace map_tiles

} // namespace camp
