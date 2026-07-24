#ifndef CAMP_MAP_TILES_WMS_H
#define CAMP_MAP_TILES_WMS_H

#include "tile_layout.h"

namespace camp
{

namespace wms
{

// [camp#118] Generate a tile layout whose tiles are fetched as per-tile WMS 1.3.0
// GetMap requests (EPSG:3857 bbox per slippy tile) instead of z/x/y paths — the
// analog of osm::generateTileLayout for WMS-only sources (nowCOAST radar, GEBCO).
// The grid is identical to the OSM one, so the layers ride the full MapTiles
// lifecycle: disk cache, #98 eviction, #99/#111 refresh + cache-buster (the
// buster's '&' branch handles the query string already present here).
//
// VERSION=1.3.0 and CRS=EPSG:3857 are deliberately hardcoded, not parameters:
// the per-tile path only works against the Web-Mercator tile grid, so the CRS
// is structural; 1.3.0 is the current WMS spec both target endpoints accept
// (see camp ADR-0012).
map_tiles::TileLayout generateWmsLayout(std::string base_url, std::string layer_id,
                                        int max_level = 19, int min_level = 0);

} // namespace wms

} // namespace camp

#endif
