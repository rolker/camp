#include "markers_manager.h"
#include "backgroundraster.h"
#include "markers.h"
#include <QTimer>

MarkersManager::MarkersManager(QWidget* parent):
  QWidget(parent)
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
  ros::NodeHandle nh;

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  for(const auto t: topic_info)
    if (t.datatype == "visualization_msgs/MarkerArray" || t.datatype == "visualization_msgs/Marker")
      if (markers_.find(t.name) == markers_.end())
      {
        markers_[t.name] = new Markers(this, background_);
        markers_[t.name]->setTopic(t.name, t.datatype);
        markers_[t.name]->setTF2Buffer(tf_buffer_);
        markers_[t.name]->setObjectName(t.name.c_str());
        if(background_)
          markers_[t.name]->setPixelSize(background_->pixelSize());
        ui_.markersLayout->addWidget(markers_[t.name]);
      }
}

void MarkersManager::setTFBuffer(tf2_ros::Buffer* buffer)
{
  tf_buffer_ = buffer;
  for(auto og: markers_)
    og.second->setTF2Buffer(buffer);
}

void MarkersManager::updateBackground(BackgroundRaster * bg)
{
  background_ = bg;
  for(auto og: markers_)
    og.second->updateBackground(bg);
}
