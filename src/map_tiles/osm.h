#ifndef CAMP_MAP_TILES_OSM_H
#define CAMP_MAP_TILES_OSM_H

#include "tile_layout.h"

namespace camp
{

namespace osm
{

// Size in pixels of tiles used with web mercator projections.
// At zoom level 0, one square tile covers the earth from 0 latitude
// to 0 latitude and maximum_latitude to -maximum_latitude.
constexpr int tile_size = 256;


map_tiles::TileLayout generateTileLayout(std::string base_url, int max_level=19, int min_level=0);

} // namespace osm

} // namespace camp

#endif
