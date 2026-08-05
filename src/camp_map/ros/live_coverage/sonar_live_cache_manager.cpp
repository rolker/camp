#include "sonar_live_cache_manager.h"

#include "../node.h"
#include "../names_manager.h"
#include "../../map/layer_list.h"
#include "sonar_live_cache_layer.h"

#include <QString>

namespace camp
{
namespace ros
{
namespace live_coverage
{

namespace
{
// The tile-stream topic suffix; the source base namespace is the topic with this
// stripped (e.g. "/cube_bathymetry/coverage_tiles" -> "/cube_bathymetry").
constexpr char kTilesSuffix[] = "/coverage_tiles";
}  // namespace

SonarLiveCacheManager::SonarLiveCacheManager(MapTool* parent):
  tools::LayerManager(parent, "Live Coverage Manager")
{
  auto topics_manager = new TopicsManager(this, "Topics");
  topics_manager->setTypeFilter({"marine_interfaces/msg/SonarVisualizationTile"});
  connect(topics_manager, &TopicsManager::namesUpdated, this,
          &SonarLiveCacheManager::updateTopics);
}

void SonarLiveCacheManager::updateTopics()
{
  auto topic_manager = firstChildOfType<TopicsManager>();
  if(!topic_manager)
    return;
  auto node_item = parentOfType<Node>();
  if(!node_item)
    return;
  auto node = node_item->node();
  if(!node)
    return;

  const std::string suffix = kTilesSuffix;
  for(const auto& topic : topic_manager->namesAndTypes())
  {
    if(node->count_publishers(topic.first) == 0)
      continue;
    bool is_tile_topic = false;
    for(const auto& type : topic.second)
      if(type == "marine_interfaces/msg/SonarVisualizationTile")
        is_tile_topic = true;
    if(!is_tile_topic)
      continue;

    // Derive the source base namespace by stripping the tile-stream suffix.
    const std::string& full = topic.first;
    if(full.size() <= suffix.size() ||
       full.compare(full.size() - suffix.size(), suffix.size(), suffix) != 0)
      continue;
    const std::string base = full.substr(0, full.size() - suffix.size());

    if(sources_.count(base))
      continue;   // already spawned a layer for this source
    auto layers = topLevelLayers();
    if(!layers)
      continue;
    auto layer = new SonarLiveCacheLayer(layers, node_item, QString::fromStdString(base));
    trackSpawnedLayer(base, layer);
  }
}

bool SonarLiveCacheManager::isSourceTracked(const std::string& base) const
{
  return sources_.count(base) > 0;
}

void SonarLiveCacheManager::trackSpawnedLayer(const std::string& base, QObject* layer)
{
  sources_.insert(base);
  // Clear the entry when the layer dies so the next updateTopics() can respawn
  // it (camp#168 — the 2026-07-23 field incident: remove made the live coverage
  // layer un-re-addable for the rest of the session). Between the operator's
  // remove (deleteLater()) and ~QObject firing `destroyed` the entry lingers;
  // the event loop drains before any human re-add, so no spurious skip in
  // practice. DirectConnection is safe: both deleteLater() destruction and this
  // erase run on the GUI thread and the lambda touches only the set.
  connect(layer, &QObject::destroyed, this,
          [this, base]() { sources_.erase(base); }, Qt::DirectConnection);
}

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp
