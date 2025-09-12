#include "marker_namespace.h"
#include "marker.h"

namespace camp
{
namespace ros
{
namespace markers
{

MarkerNamespace::MarkerNamespace(MapItem* parent, Node* node, QString marker_namespace):
  Layer(parent, node, marker_namespace)
{
}

void MarkerNamespace::updateMarker(const MarkerData& data)
{
  auto markers_map = markers();
  if(data.marker.action == visualization_msgs::msg::Marker::DELETEALL)
  {
    for(auto m: markers_map)
      m.second->updateMarker(data);
    return;
  }

  Marker* marker = nullptr;
  if(markers_map.find(data.marker.id) == markers_map.end())
    marker = new Marker(this, node_, data.marker.id);
  else
    marker = markers_map[data.marker.id];
  marker->updateMarker(data);
}

std::map<uint32_t, Marker*> MarkerNamespace::markers() const
{
  std::map<uint32_t, Marker*> markers;
  for(auto item: childItems())
  {
    auto marker = qgraphicsitem_cast<Marker*>(item);
    if(marker)
      markers[marker->id()] = marker;
  }
  return markers;
}

} // namespace markers
} // namespace ros
} // namespace camp
