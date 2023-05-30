#include "tile_address.h"

#include <stdexcept>
#include <QString>

namespace map_tiles
{

const uint8_t TileAddress::max_zoom_level;

TileAddress::TileAddress(uint8_t zoom_level, QPoint index):
  zoom_level_(zoom_level), index_(index)
{
}

TileAddress TileAddress::parent() const
{
  if(zoom_level_ == 0)
    throw std::out_of_range("No parent. TileAddress zoom level is 0");
  return TileAddress(zoom_level_-1, QPoint(index_.x()/2, index_.y()/2));
}

TileAddress TileAddress::child(uint8_t x, uint8_t y) const
{
  if(x > 1 || y > 1)
    throw std::out_of_range("x="+std::to_string(x)+" and y="+std::to_string(y)+" are invalid TileAddress child coordinates. Each must be 0 or 1");
  return TileAddress(zoom_level_+1, QPoint(index_.x()*2+x, index_.y()*2+y));
}

TileAddress::operator std::string() const
{
  return std::to_string(zoom_level_)+"/"+std::to_string(index_.x())+"/"+std::to_string(index_.y())+".png";
}

TileAddress::operator QString() const
{
  return std::string(*this).c_str();
}

uint8_t TileAddress::zoomLevel() const
{
  return zoom_level_;
}

const QPoint& TileAddress::index() const
{
  return index_;
}

bool TileAddress::operator==(const TileAddress& other) const
{
  return zoom_level_ == other.zoom_level_ && index_ == other.index_;
}

bool TileAddress::descendentOf(const TileAddress& potential_ancestor) const
{
  auto i = *this;
  while(i.zoom_level_ > 0)
  {
    i = i.parent();
    if(i == potential_ancestor)
      return true;
  }
  return false;
}

} // namepsace map_tiles
