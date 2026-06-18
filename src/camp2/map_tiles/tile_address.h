#ifndef MAP_TILES_TILE_ADDRESS_H
#define MAP_TILES_TILE_ADDRESS_H

#include <QPoint>
#include <string>
#include <QMetaType>
#include "tile_layout.h"

namespace camp
{
  
namespace map_tiles
{

// Represents the zoom level and index of a tile
// in tiled maps.
// https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
class TileAddress
{
public:
  // [#99] epoch is a per-refresh layout generation counter (see MapTiles). It is
  // part of address *identity* (operator==) but NOT of the map ordering
  // (operator<), so two addresses that differ only by epoch sort equal yet
  // compare unequal — this lets MapTiles::tileLoaded reject an in-flight,
  // pre-refresh pixmap that lands on a rebuilt same-position tile after a refresh.
  TileAddress(const TileLayout* layout = nullptr, uint8_t zoom_level = 0, QPoint index = QPoint(), quint64 epoch = 0);
  ~TileAddress() = default;
  TileAddress(const TileAddress& other) = default;

  // Returns the address as a path compenent suitable as part
  // of a file path or url.
  // The format is "zoom/x/y.png"
  operator std::string() const;

  // QString version of std::string operator.
  operator QString() const;

  std::string url() const;

  uint8_t zoomLevel() const;
  const QPoint &index() const;

  bool operator==(const TileAddress &other) const;

  bool operator<(const TileAddress &other) const;

  const TileLayout& tileLayout() const;

  double scale() const;
  QPointF topLeftCorner() const;

private:
  const TileLayout* layout_;
  uint8_t zoom_level_ = 0;
  QPoint index_;
  // [#99] Per-refresh layout generation. Compared by operator== (identity), not
  // by operator< (ordering). Default 0 for non-refreshing layers.
  quint64 epoch_ = 0;

};

} // namespace map_tiles

} // namespace camp

Q_DECLARE_METATYPE(camp::map_tiles::TileAddress);

#endif