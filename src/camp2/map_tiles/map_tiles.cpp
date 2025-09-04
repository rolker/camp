#include "map_tiles.h"
#include <QPainter>
#include "tile.h"
#include <cmath>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "cached_tile_loader.h"
#include <QDir>
#include <QStyleOptionGraphicsItem>
#include <set>
#include "wmts/capabilities.h"

namespace camp
{

namespace map_tiles
{

MapTiles::MapTiles(map::MapItem* parentItem, const QString& label, const TileLayout& tile_layout):
  map::Layer(parentItem, label), tile_layout_(tile_layout)
{
  tile_loader_ = new CachedTileLoader(this);

  connect(tile_loader_, &CachedTileLoader::pixmapLoaded, this, &MapTiles::tileLoaded);

  auto dir = QDir::home().filePath(".CCOMAutonomousMissionPlanner/map_tiles/"+label);
  tile_loader_->setCachePath(dir);
  setLayout(tile_layout);
}

QRectF MapTiles::boundingRect() const
{
  return childrenBoundingRect();
}

void MapTiles::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  auto lod = QStyleOptionGraphicsItem::levelOfDetailFromTransform(painter->worldTransform());

  // scale up view 
  lod /= 2.0;

  auto wt = painter->worldTransform();
  auto window = painter->window();
  QPointF top_left((window.x()-wt.m31())/wt.m11(), (window.y()-wt.m32())/wt.m22());
  QPointF bottom_right(top_left.x()+window.width()/wt.m11(), top_left.y()+window.height()/wt.m22());

  int level_number = 0;
  int render_level = tile_layout_.zoom_levels.size()-1;

  std::set<TileAddress> visible_tiles;

  for(const auto& level: tile_layout_.zoom_levels)
  {
    if(level.scale*lod < 2.0)
    {
      render_level = level_number;
      break;
    }
    level_number++;
  }

  if(render_level >= 0)
  {
    const auto& level = tile_layout_.zoom_levels[render_level];
    auto start_x_index = std::max(0,std::min(level.matrix_width,int(floor( (top_left.x() - level.top_left_corner.x())/(level.tile_width*level.scale)))));
    auto end_x_index = std::max(0,std::min(level.matrix_width,int(ceil(( bottom_right.x() - level.top_left_corner.x())/(level.tile_width*level.scale)))));
    auto start_y_index = std::max(0,std::min(level.matrix_height,int(floor( (level.top_left_corner.y()-top_left.y()))/(level.tile_height*level.scale))));
    auto end_y_index = std::max(0,std::min(level.matrix_height,int(ceil( (level.top_left_corner.y()-bottom_right.y())/(level.tile_height*level.scale)))));

    for(int row = start_y_index; row <= end_y_index && row < level.matrix_height; row++)
      for(int col = start_x_index; col <= end_x_index && col < level.matrix_width; col++)
      {
        TileAddress address(&tile_layout_, render_level, QPoint(col, row));
        if(tiles_.find(address) == tiles_.end() || tiles_[address] == nullptr)
        {
          tiles_[address] = new Tile(address, this);
          tile_loader_->load(address);
        }
        tiles_[address]->setVisible(true);
        visible_tiles.insert(address);
      }
  }

  for(auto tile: tiles_)
    if(visible_tiles.find(tile.first) == visible_tiles.end())
      tile.second->setVisible(false);
}

void MapTiles::setLayout(const TileLayout& tile_layout)
{
  for(auto tile: tiles_)
    if(tile.second)
      delete tile.second;
  tiles_.clear();
  tile_layout_ = tile_layout;
  if(!tile_layout_.zoom_levels.empty())
  {
    const auto &top_level = tile_layout_.zoom_levels.front();
    for(int row = 0; row < top_level.matrix_height; row++)
      for(int col = 0; col < top_level.matrix_width; col++)
      {
        TileAddress address(&tile_layout_, 0, QPoint(col, row));
        tiles_[address] = new Tile(address, this);
        tile_loader_->load(address);
      }
  }
}

void MapTiles::setLayoutFromWMTS(const wmts::Capabilities &capabilites, QString layer_id, QString tile_matrix_set)
{
  wmts_capabilites_ = &capabilites;
  wmts_layer_id_ = layer_id;
  wmts_tile_matrix_set_ = tile_matrix_set;
  connect(wmts_capabilites_, &wmts::Capabilities::ready, this, &MapTiles::wmtsCapabilitiesReady);
}

void MapTiles::wmtsCapabilitiesReady()
{
  setLayout(wmts_capabilites_->getLayout(wmts_layer_id_, wmts_tile_matrix_set_));
}

void MapTiles::updateViewScale(double view_scale)
{
  //updateViewport(view_context_.viewport);
}

void MapTiles::loadTile(TileAddress tile_address)
{
  tile_loader_->load(tile_address);
}

void MapTiles::tileLoaded(QPixmap pixmap, TileAddress tile_address)
{
  if(tiles_.find(tile_address) != tiles_.end() && tiles_[tile_address] != nullptr)
    // The < operator used for map does not consider layout, but the == operator does.
    // This is to make sure an old pixmap loading before a setLayout call doesn't make
    // it to a new layout's tile.
    if(tiles_[tile_address]->address() == tile_address) 
      tiles_[tile_address]->updatePixmap(pixmap);
}

} // namespace map_tiles

} // namespace camp
