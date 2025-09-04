#include "layer.h"

#include <QDebug>

namespace camp
{

namespace wmts
{

Layer::Layer(QDomElement layer_element)
{
  auto id_element = layer_element.firstChildElement("Identifier");
  id = id_element.text();
  title = id;
  auto title_element = layer_element.firstChildElement("Title");
  if(!title_element.isNull())
    title = title_element.text();

  auto style_elements = layer_element.elementsByTagName("Style");
  for(int i = 0; i < style_elements.size(); i++)
  {
    auto style = style_elements.at(i);
    styles.push_back(style.firstChildElement("Identifier").text());
    auto is_default = style.toElement().attribute("isDefault", "false");
    if(is_default == "true")
      default_style = styles.back();
  }
  auto format_elements = layer_element.elementsByTagName("Format");
  for(int i = 0; i < format_elements.size(); i++)
    if(format_elements.at(i).toElement().text() == "image/png")
      has_png_format = true;

  auto tmsl_elements = layer_element.elementsByTagName("TileMatrixSetLink");
  for(int i = 0; i < tmsl_elements.size(); i++)
  {
    tile_matrix_set_links.push_back(tmsl_elements.at(i).firstChildElement("TileMatrixSet").text());
  }

  auto resource_urls_elements = layer_element.elementsByTagName("ResourceURL");
  for(int i = 0; i < resource_urls_elements.size(); i++)
  {
    auto ru_element = resource_urls_elements.at(i).toElement();
    if(ru_element.attribute("format").contains("image/png") && ru_element.attribute("resourceType") == "tile")
    {
      resource_url_template = ru_element.attribute("template");
      break;
    }
  }
}

} // namespace wmts

} // namespace camp
