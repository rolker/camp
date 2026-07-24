#include "wms.h"

#include "osm.h"

namespace camp
{

namespace wms
{

map_tiles::TileLayout generateWmsLayout(std::string base_url, std::string layer_id,
                                        int max_level, int min_level)
{
  // Same Web-Mercator grid as the OSM layout — only the URL template differs.
  // Starting from generateTileLayout keeps the two grids identical by
  // construction (zoom levels, scales, matrix sizes).
  map_tiles::TileLayout layout = osm::generateTileLayout(base_url, max_level, min_level);
  layout.url_static_parts.clear();
  layout.url_variable_keys.clear();

  // WIDTH/HEIGHT come from osm::tile_size — the same constant the bbox math in
  // TileLayout::getUrl uses via tile_width/height — so the raster size and the
  // bbox cannot drift apart.
  const std::string size = std::to_string(osm::tile_size);
  layout.url_static_parts.push_back(
    base_url + "?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&BBOX=");
  layout.url_variable_keys.push_back("WMS_BBOX");
  layout.url_static_parts.push_back(
    "&CRS=EPSG:3857&WIDTH=" + size + "&HEIGHT=" + size +
    "&FORMAT=image/png&TRANSPARENT=TRUE&LAYERS=" + layer_id);

  return layout;
}

} // namespace wms

} // namespace camp
