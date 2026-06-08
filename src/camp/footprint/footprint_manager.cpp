#include "footprint_manager.h"

#include <QTimer>

#include "footprint/footprint.h"

FootprintManager::FootprintManager(QObject* parent):
  camp_ros::ROSObject(parent)
{
  scan_timer_ = new QTimer(this);
  connect(scan_timer_, &QTimer::timeout, this, &FootprintManager::scanForSources);
  scan_timer_->start(1000);
}

void FootprintManager::scanForSources()
{
  if(!node_)
    return;

  auto topics = node_->get_topic_names_and_types();
  for(const auto& topic: topics)
  {
    const auto& name = topic.first;
    // Match nav2's `.../local_costmap/published_footprint` robustly to whatever
    // namespace prefix the udp_bridge applies (e.g. /bizzy/...).
    if(name.find("published_footprint") == std::string::npos)
      continue;
    if(footprints_.find(name) != footprints_.end())
      continue;
    for(const auto& type: topic.second)
    {
      if(type == "geometry_msgs/msg/PolygonStamped")
      {
        auto* fp = new Footprint(this, m_anchor);
        fp->setObjectName(name.c_str());
        fp->nodeStarted(node_, transform_buffer_);
        fp->setTopic(name);
        footprints_[name] = fp;
        break;
      }
    }
  }
}

void FootprintManager::updateBackground()
{
  for(auto& entry: footprints_)
    entry.second->updateBackground();
}
