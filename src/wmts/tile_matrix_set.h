#ifndef CAMP_WMTS_TILE_MATRIX_SET_H
#define CAMP_WMTS_TILE_MATRIX_SET_H

#include <QDomElement>
#include <vector>
#include <QPointF>

namespace camp
{

namespace wmts
{

struct TileMatrix
{
  QString id;
  double scale_denominator;
  QPointF top_left_corner;
  int tile_width;
  int tile_height;
  int matrix_width;
  int matrix_height;
};

struct TileMatrixSet
{
  TileMatrixSet(QDomElement tms_element);

  QString id;
  QString supported_crs;
  std::vector<TileMatrix> tile_matrices;
};

} // namespace wmts

} // namespace camp

#endif
