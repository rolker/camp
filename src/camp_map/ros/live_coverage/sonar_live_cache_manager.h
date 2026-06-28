#ifndef CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_CACHE_MANAGER_H
#define CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_CACHE_MANAGER_H

#include "../../tools/layer_manager.h"

#include <map>
#include <string>

namespace camp
{
namespace ros
{
namespace live_coverage
{

/// [camp#121] Discovers boat-side live-coverage sources and spawns one
/// `SonarLiveCacheLayer` per source. Mirrors `GridManager`'s topic-discovery
/// pattern: a child `TopicsManager` filtered to `SonarVisualizationTile`; on each
/// graph update, every `<base>/coverage_tiles` topic with an active publisher gets
/// a layer (deduped by base namespace).
///
/// Discovery is automatic, but the spawned layer is *inactive* — it does not
/// subscribe to the best-effort tile stream until the operator enables it
/// (ADR-0006 D5). The expensive subscription is the layer's decision, gated on its
/// persisted per-source enabled flag; the manager only ensures the layer exists.
class SonarLiveCacheManager: public tools::LayerManager
{
  Q_OBJECT
public:
  SonarLiveCacheManager(MapTool* parent);

public slots:
  void updateTopics();

private:
  // base namespace -> spawned, so a topic reappearing (DDS re-discovery) doesn't
  // spawn a duplicate; the existing layer's subscriptions reconnect on their own.
  std::map<std::string, bool> sources_;
};

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp

#endif
