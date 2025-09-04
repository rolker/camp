#include "tile_address.h"

#include <stdexcept>
#include <QString>
#include <tuple>

namespace camp
{

namespace map_tiles
{

TileAddress::TileAddress(const TileLayout* layout, uint8_t zoom_level, QPoint index):
  layout_(layout), zoom_level_(zoom_level), index_(index)
{
}


TileAddress::operator std::string() const
{
  return std::to_string(zoom_level_)+"/"+std::to_string(index_.x())+"/"+std::to_string(index_.y())+".png";
}

TileAddress::operator QString() const
{
  return std::string(*this).c_str();
}

std::string TileAddress::url() const
{
  return layout_->getUrl(*this);
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
  return zoom_level_ == other.zoom_level_ && index_ == other.index_ && layout_ == other.layout_;
}

bool TileAddress::operator<(const TileAddress& other) const
{
  auto i = index_;
  auto oi = other.index_;
  return std::tie(zoom_level_, i.ry(), i.rx()) < std::tie(other.zoom_level_, oi.ry(), oi.rx());
}

const TileLayout& TileAddress::tileLayout() const
{
  return *layout_;
}

double TileAddress::scale() const
{
  return layout_->zoom_levels[zoom_level_].scale;
}

QPointF TileAddress::topLeftCorner() const
{
  auto level = layout_->zoom_levels[zoom_level_];
  return QPointF(level.top_left_corner.x()+index_.x()*level.scale*level.tile_width, level.top_left_corner.y()-index_.y()*level.scale*level.tile_height);
}

} // namepsace map_tiles

} // camp
