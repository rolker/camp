#include "map_tiles.h"
#include <QPainter>
#include "tile.h"
#include <cmath>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "cached_tile_loader.h"
#include <QDir>

#include <QDebug>

namespace map_tiles
{

MapTiles::MapTiles(map::MapItem* parentItem, const QString& label):
  map::Layer(parentItem, label)
{
  tile_loader_ = new CachedTileLoader(this);

  connect(tile_loader_, &CachedTileLoader::pixmapLoaded, this, &MapTiles::tileLoaded);

  auto dir = QDir::home().filePath(".CCOMAutonomousMissionPlanner/map_tiles/"+label);
  tile_loader_->setCachePath(dir);

  top_tile_ = new Tile(this);

  // size of a pixel relative to a meter
  double scale = web_mercator::earth_radius_at_equator*2.0*M_PI/double(web_mercator::tile_size);

  // tile raster origin is upper left corner, so flip y to make increasing raster y go down in latitude
  top_tile_->setTransform(QTransform::fromScale(scale, -scale), true);

  double half_earth_circumference = web_mercator::earth_radius_at_equator*M_PI;
  top_tile_->setPos(-half_earth_circumference, half_earth_circumference);
}

QRectF MapTiles::boundingRect() const
{
  return  childrenBoundingRect();
}

void MapTiles::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
}

bool MapTiles::flipY() const
{
  return flip_y_;
}

void MapTiles::setFlipY(bool flipped)
{
  flip_y_ = flipped;
  top_tile_->updateLayout(flipped);
}

uint8_t MapTiles::minimumZoomLevel() const
{
  return minimum_zoom_level_;
}

void MapTiles::setMinimumZoomLevel(uint8_t level)
{
  minimum_zoom_level_ = level;
  update();
}

uint8_t MapTiles::maximumZoomLevel() const
{
  return maximum_zoom_level_;
}

void MapTiles::setMaximumZoomLevel(uint8_t level)
{
  maximum_zoom_level_ = level;
  update();
}

void MapTiles::updateViewScale(double view_scale)
{
  //updateViewport(view_context_.viewport);
}

void MapTiles::setBaseUrl(QString base_url)
{
  tile_loader_->setBaseUrl(base_url);
}

void MapTiles::loadTile(TileAddress tile_address)
{
  tile_loader_->load(tile_address);
}

void MapTiles::tileLoaded(QPixmap pixmap, TileAddress tile_address)
{
  auto tile = top_tile_->find(tile_address);
  if(tile)
    tile->updatePixmap(pixmap);
}

} // namespace map_tiles
