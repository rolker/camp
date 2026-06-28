#include "tile_matrix_set.h"

#include "layer.h"
#include <QStringList>

namespace camp
{

namespace wmts
{

TileMatrixSet::TileMatrixSet(QDomElement tms_element)
{
  id = tms_element.firstChildElement("Identifier").text();
  supported_crs = tms_element.firstChildElement("SupportedCRS").text();
  auto tile_matrix_elements = tms_element.elementsByTagName("TileMatrix");
  for(int i = 0; i < tile_matrix_elements.size(); i++)
  {
    auto tile_matix_element = tile_matrix_elements.at(i).toElement();
    TileMatrix tm;
    tm.id = tile_matix_element.firstChildElement("Identifier").text();
    tm.scale_denominator = tile_matix_element.firstChildElement("ScaleDenominator").text().toDouble();
    auto top_left_strings = tile_matix_element.firstChildElement("TopLeftCorner").text().split(" ");
    tm.top_left_corner.rx() = top_left_strings[0].toDouble();
    tm.top_left_corner.ry() = top_left_strings[1].toDouble();
    tm.tile_width = tile_matix_element.firstChildElement("TileWidth").text().toInt();
    tm.tile_height = tile_matix_element.firstChildElement("TileHeight").text().toInt();
    tm.matrix_width = tile_matix_element.firstChildElement("MatrixWidth").text().toInt();
    tm.matrix_height = tile_matix_element.firstChildElement("MatrixHeight").text().toInt();
    tile_matrices.push_back(tm);
  }
}



} // namespace wmts

} // namespace camp
