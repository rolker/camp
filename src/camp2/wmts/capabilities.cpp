#include "capabilities.h"

#include "layer.h"
#include "main/cached_file_loader.h"
#include <QDomDocument>
#include <QRegularExpression>

#include <QDebug>

namespace camp
{

namespace wmts
{

Capabilities::Capabilities(QString label,QObject* parent):
  QObject(parent), label_(label)
{

}

void Capabilities::setUrl(QString url)
{
  CachedFileClient* client = new CachedFileClient(this);
  connect(client, &CachedFileClient::dataLoaded, this, &Capabilities::dataLoaded);
  CachedFileLoader::get()->load(url, "wmts/"+label_+"/capabilities.xml", client);
}

void Capabilities::dataLoaded(QByteArray &data, CachedFileClient* client)
{
  QDomDocument document;
  document.setContent(data, true);

  auto root = document.documentElement();
  if(root.tagName() != "Capabilities")
  {
    qDebug() << "expected top level to be Capabilities, got" << root.tagName();
    return;
  
  }
  auto contents = root.firstChildElement("Contents");
  if(contents.isNull())
  {
    qDebug() << "Contents element not found.";
    return;
  }

  auto layers = contents.elementsByTagName("Layer");
  for(int i = 0; i < layers.count(); i++)
  {
    Layer layer(layers.at(i).toElement());
    layers_.push_back(layer);
  }

  auto tile_matrix_set_elements = contents.elementsByTagName("TileMatrixSet");
  for(int i = 0; i < tile_matrix_set_elements.size(); i++)
  {
    TileMatrixSet tms(tile_matrix_set_elements.at(i).toElement());
    tile_matrix_sets_.push_back(tms);
  }

  emit ready();
}

map_tiles::TileLayout Capabilities::getLayout(QString layer_id, QString tile_matrix_set) const
{
  map_tiles::TileLayout layout;
  for(const auto &layer: layers_)
    if(layer_id.isEmpty() || layer.id == layer_id)
    {
      auto parts = layer.resource_url_template.split(QRegularExpression("[{}]"));
      QString pending_static_string;
      for(int i = 0; i < parts.size(); i += 2)
        if(i+1 < parts.size())
        {
          auto key = parts[i+1];
          if(key == "style" || key == "Style") // specs say "style", example from noaa arcgis uses "Style"
          {
            auto style = layer.default_style;
            if(style.isEmpty())
              style = layer.styles.front();
            pending_static_string += parts[i]+style;
          }
          else if(key == "TileMatrixSet")
          {
            // \todo check that passed in tile_matrix_set exists.
            if(tile_matrix_set.isNull())
              tile_matrix_set = layer.tile_matrix_set_links.front();
            pending_static_string += parts[i]+tile_matrix_set;
          }
          else
          {
            layout.url_static_parts.push_back((pending_static_string+parts[i]).toStdString());
            layout.url_variable_keys.push_back(key.toStdString());
            pending_static_string.clear();
          }
        }
        else
          layout.url_static_parts.push_back((pending_static_string+parts[i]).toStdString());
      break;
    }
  for(const auto& tms: tile_matrix_sets_)
    if(tms.id == tile_matrix_set)
    {
      // \todo check for correct crs
      for(const auto& tm: tms.tile_matrices)
      {
        map_tiles::TileLayout::ZoomLevel zoom_level;
        zoom_level.id = tm.id.toStdString();
        // wmts scale based on 0.28mm pixel size
        zoom_level.scale = tm.scale_denominator * 0.00028;
        zoom_level.top_left_corner = tm.top_left_corner;
        zoom_level.tile_width = tm.tile_width;
        zoom_level.tile_height = tm.tile_height;
        zoom_level.matrix_width = tm.matrix_width;
        zoom_level.matrix_height = tm.matrix_height;
        layout.zoom_levels.push_back(zoom_level);
      }
    }
  return layout;
}

} // namespace wmts

} // namespace camp
