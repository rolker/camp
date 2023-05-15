#include "map_tiles.h"
#include <QPainter>
#include "tile.h"
#include <cmath>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "cached_tile_loader.h"
#include <QDir>

namespace map_tiles
{

MapTiles::MapTiles(QGraphicsItem *parentItem):
  QGraphicsObject(parentItem)
{
  tile_loader_ = new CachedTileLoader(this);

  connect(tile_loader_, &CachedTileLoader::pixmapLoaded, this, &MapTiles::tileLoaded);

  top_tile_ = new Tile(view_context_, this);
}

QRectF MapTiles::boundingRect() const
{
  return  childrenBoundingRect();
}

void MapTiles::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,QWidget *widget)
{
  painter->save();
  painter->setRenderHint(QPainter::SmoothPixmapTransform);
  double scale = painter->transform().m11();
  painter->restore();
}

void MapTiles::updateViewport(MapView::Viewport viewport)
{
  view_context_.viewport = viewport;
  if(viewport.pixels_per_map_unit > 0.0)
  // todo, change max zoom of 18 to a parameter
  //   view_context_.current_zoom_level = std::min(std::max(ui_.minimumLevelSpinBox->value(), int(floor(log2(viewport.pixels_per_map_unit/ui_.viewScaleDoubleSpinBox->value())))), 18);
  // else
  //   view_context_.current_zoom_level = ui_.minimumLevelSpinBox->value();

    view_context_.current_zoom_level = std::min(std::max(0, int(floor(log2(viewport.pixels_per_map_unit/1.5)))), 18);
  else
    view_context_.current_zoom_level = 0;


  top_tile_->updateView();
}

void MapTiles::updateMinimumZoomLevel(int level)
{
  updateViewport(view_context_.viewport);
}

void MapTiles::updateViewScale(double view_scale)
{
  updateViewport(view_context_.viewport);
}

void MapTiles::setYDirection(bool flipped)
{
  view_context_.flip_y = flipped;
  top_tile_->updateLayout();
}

void MapTiles::setLabel(QString label)
{
  auto dir = QDir::home().filePath(".CCOMAutonomousMissionPlanner/map_tiles/"+label);
  //ui_.localPathLineEdit->setText(dir);
  tile_loader_->setCachePath(dir);
  //if(ui_.labelLineEdit->text() != label)
    //ui_.labelLineEdit->setText(label);
}

void MapTiles::labelEditingFinished()
{
  //setLabel(ui_.labelLineEdit->text());
}

void MapTiles::setBaseUrl(QString base_url)
{
  tile_loader_->setBaseUrl(base_url);
  //if(ui_.baseURLLineEdit->text() != base_url)
  //  ui_.baseURLLineEdit->setText(base_url);
}

void MapTiles::baseUrlEditingFinished()
{
  //setBaseUrl(ui_.baseURLLineEdit->text());
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
