#ifndef CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_CACHE_MANAGER_H
#define CAMP_ROS_LIVE_COVERAGE_SONAR_LIVE_CACHE_MANAGER_H

#include "../../tools/layer_manager.h"

#include <set>
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

  /// True while a layer for `base` is tracked (spawned and not yet destroyed).
  /// Introspection + test seam (camp#168).
  bool isSourceTracked(const std::string& base) const;

public slots:
  void updateTopics();

protected:
  /// Register a spawned layer for `base`: insert into `sources_` and arrange
  /// for the entry to clear when the layer is destroyed, so the source can be
  /// re-added after an operator removes the layer (camp#168). Factored from
  /// `updateTopics()` so the tracking contract is testable without the live
  /// `Node` ancestor that topic discovery requires.
  void trackSpawnedLayer(const std::string& base, QObject* layer);

private:
  // Base namespaces with a live spawned layer, so a topic reappearing (DDS
  // re-discovery) doesn't spawn a duplicate. The entry is erased when the
  // layer is destroyed (operator remove -> deleteLater, or shutdown), so the
  // next updateTopics() can respawn it (camp#168).
  std::set<std::string> sources_;
};

}  // namespace live_coverage
}  // namespace ros
}  // namespace camp

#endif
