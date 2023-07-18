#ifndef MAP_TILES_TILE_LAYOUT_H
#define MAP_TILES_TILE_LAYOUT_H


#include <string>
#include <vector>
#include <QPointF>
#include <QRectF>

namespace camp
{

namespace map_tiles
{

class TileAddress;

struct TileLayout
{
  struct ZoomLevel
  {
    std::string id;

    // size of a pixel in map units
    double scale;

    // top left corner in projected coordinates
    QPointF top_left_corner;

    // width of a tile in pixels
    int tile_width;
    // height of a tile in pixels
    int tile_height;

    // number of tile columns
    int matrix_width;

    // number of tile rows
    int matrix_height; 


    QRectF boundingRect() const;
  };

  std::vector<ZoomLevel> zoom_levels;

  // decomposed url template is made up of static parts
  // and keys for the changing parts (zoom, row, col)
  // Keys used are as specified in WMTS template specs.
  // "TileMatrix" for zoom level, "TileRow" for tow and "TileCol"
  // for column.
  std::vector<std::string> url_static_parts;
  std::vector<std::string> url_variable_keys;


  QRectF boundingRect() const;
  std::string getUrl(const TileAddress &) const;
};

} // namespace map_tiles


} // namespace camp

#endif
