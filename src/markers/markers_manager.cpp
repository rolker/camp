#include "markers_manager.h"
#include "backgroundraster.h"
#include "markers.h"
#include <QTimer>

MarkersManager::MarkersManager(QWidget* parent):
  camp_ros::ROSWidget(parent)
{
  ui_.setupUi(this);

  scan_timer_ = new QTimer(this);
  connect(scan_timer_, &QTimer::timeout, this, &MarkersManager::scanForSources);
  scan_timer_->start(1000);
}

MarkersManager::~MarkersManager()
{

}

void MarkersManager::scanForSources()
{
  if(node_)
  {
    auto topics = node_->get_topic_names_and_types();
    for(auto topic: topics)
    {
      auto name = topic.first;
      for(auto topic_type: topic.second)
      {
        if (topic_type == "visualization_msgs/MarkerArray" || topic_type == "visualization_msgs/Marker")
        {
          if(markers_.find(name) == markers_.end())
          {
            markers_[name] = new Markers(this, background_);
            markers_[name]->nodeStarted(node_, transform_buffer_);
            markers_[name]->setTopic(name, topic_type);
            markers_[name]->setObjectName(name.c_str());
            if(background_)
              markers_[name]->setPixelSize(background_->pixelSize());
            ui_.markersLayout->addWidget(markers_[name]);
          }
        }
      }
    }
  }
}


void MarkersManager::updateBackground(BackgroundRaster * bg)
{
  background_ = bg;
  for(auto og: markers_)
    og.second->updateBackground(bg);
}
