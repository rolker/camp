#include "tile_layout.h"
#include "tile_address.h"

#include <QString>
#include <QUrl>

#include <iomanip>
#include <sstream>

namespace camp
{

namespace map_tiles
{

QRectF TileLayout::boundingRect() const
{
  if(!zoom_levels.empty())
    return zoom_levels.front().boundingRect();
  return {};
}

QRectF TileLayout::ZoomLevel::boundingRect() const
{
  return QRectF(top_left_corner, QSizeF(scale*tile_width*matrix_width, scale*tile_height*matrix_height));
}

std::string TileLayout::getUrl(const TileAddress& address) const
{
  std::string url;
  for(std::size_t i = 0; i < std::max(url_static_parts.size(), url_variable_keys.size()); i++)
  {
    if(i < url_static_parts.size())
      url += url_static_parts[i];
    if(i < url_variable_keys.size())
    {
      auto key = url_variable_keys[i];
      // [camp#178] GeoServer GWC names its tile matrices <gridset>:<z> (e.g.
      // EPSG:3857:1), so emit the declared zoom-level identifier parsed from
      // the WMTS capabilities. Fall back to the bare numeric index when no id
      // was populated (bare-numeric servers), so those keep working.
      if(key == "TileMatrix")
      {
        const std::string& id = zoom_levels[address.zoomLevel()].id;
        const std::string value = id.empty() ? std::to_string(address.zoomLevel()) : id;
        // [camp#178 review R1] The id is a raw server-controlled string (the
        // WMTS <ows:Identifier>), so percent-encode it before splicing into the
        // URL path — a hostile capabilities document must not be able to inject
        // path/query/fragment delimiters. ':' is a legal path-segment character
        // and load-bearing for GeoServer GWC ids (EPSG:3857:<z>), so it is
        // excluded from encoding; the bare-numeric fallback is digits-only and
        // passes through unchanged. TileRow/TileCol below are locally-computed
        // integers, not server data, so they need no encoding.
        url += QUrl::toPercentEncoding(QString::fromStdString(value), ":").toStdString();
      }
      if(key == "TileRow")
        url += std::to_string(address.index().y());
      if(key == "TileCol")
        url += std::to_string(address.index().x());
      // [camp#118] WMS per-tile GetMap: expand to this tile's EPSG:3857 bbox
      // (minx,miny,maxx,maxy — WMS 1.3.0 axis order for EPSG:3857 is easting,
      // northing). Two decimal places (centimeter precision in Web-Mercator
      // meters) keeps URLs stable and short.
      if(key == "WMS_BBOX")
      {
        const auto& level = zoom_levels[address.zoomLevel()];
        const QPointF top_left = address.topLeftCorner();
        const double min_x = top_left.x();
        const double max_y = top_left.y();
        const double max_x = min_x + level.scale*level.tile_width;
        const double min_y = max_y - level.scale*level.tile_height;
        std::ostringstream bbox;
        bbox << std::fixed << std::setprecision(2)
             << min_x << ',' << min_y << ',' << max_x << ',' << max_y;
        url += bbox.str();
      }
    }
  }
  return url;
}


} // namespace map_tiles

} // namespace camp
