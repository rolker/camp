#ifndef CAMP_WMTS_LAYER_H
#define CAMP_WMTS_LAYER_H

#include <QDomElement>
#include <vector>

namespace camp
{

namespace wmts
{

struct Layer
{
  Layer(QDomElement layer_element);

  QString title;
  QString id;


  std::vector<QString> styles;
  QString default_style;

  bool has_png_format = false;

  // \TODO add TileMatrixSetLimits to links
  std::vector<QString> tile_matrix_set_links;


  QString resource_url_template;
};

} // namespace wmts

} // namespace camp

#endif
