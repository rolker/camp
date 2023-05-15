#ifndef MAP_TILES_TILE_ADDRESS_H
#define MAP_TILES_TILE_ADDRESS_H

#include <QPoint>
#include <string>
#include <QMetaType>

namespace map_tiles
{

// Represents the zoom level and index of a tile
// in tiled maps.
// https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
class TileAddress
{
public:
  TileAddress(uint8_t zoom_level = 0, QPoint index = QPoint());
  ~TileAddress() = default;
  TileAddress(const TileAddress& other) = default;

  // Returns the parent address or throws std::out_of_range
  // if already at zoom level 0.
  TileAddress parent() const;

  // Returns one of the 4 child tile addresses.
  // Each of the x and y must be in range from 0 to 1, otherwise
  // a std::out_of_range exception will be thrown.
  TileAddress child(uint8_t x, uint8_t y) const;

  // Returns the address as a path compenent suitable as part
  // of a file path or url.
  // The format is "zoom/x/y.png"
  operator std::string() const;

  // QString version of std::string operator.
  operator QString() const;

  uint8_t zoomLevel() const;
  const QPoint &index() const;

  bool operator==(const TileAddress &other) const;

  bool descendentOf(const TileAddress &potential_ancestor) const;

private:
  uint8_t zoom_level_ = 0;
  QPoint index_;
};

} // namespace map_tiles

Q_DECLARE_METATYPE(map_tiles::TileAddress);

#endif