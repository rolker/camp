#include "osm.h"

#include "map_view/web_mercator.h"

namespace camp
{

namespace osm
{

// Generate a tile layout compatible with Open Street Map tile layouts.
map_tiles::TileLayout generateTileLayout(std::string base_url, int max_level, int min_level)
{
  // size of a pixel relative to a meter
  double scale = web_mercator::earth_radius_at_equator*2.0*M_PI/double(tile_size);

  double half_earth_circumference = web_mercator::earth_radius_at_equator*M_PI;
  QPointF top_left_corner(-half_earth_circumference, half_earth_circumference);

  int tile_count = 1;

  map_tiles::TileLayout layout;

  for(int zoom_level = min_level; zoom_level <= max_level; zoom_level++)
  {
    map_tiles::TileLayout::ZoomLevel level;
    level.id = std::to_string(zoom_level);
    level.scale = scale;
    level.top_left_corner = top_left_corner;

    level.tile_width = tile_size;
    level.tile_height = tile_size;
    level.matrix_width = tile_count;
    level.matrix_height = tile_count;

    layout.zoom_levels.push_back(level);

    scale /= 2.0;
    tile_count *= 2;
  }

  layout.url_static_parts.push_back(base_url);
  layout.url_variable_keys.push_back("TileMatrix");
  layout.url_static_parts.push_back("/");
  layout.url_variable_keys.push_back("TileCol");
  layout.url_static_parts.push_back("/");
  layout.url_variable_keys.push_back("TileRow");
  layout.url_static_parts.push_back(".png");

  return layout;
}

} // namespace osm

} // namespace camp
