#include "grid_manager.h"
#include "backgroundraster.h"
#include "grid.h"
#include <QTimer>

GridManager::GridManager(QWidget* parent):
  QWidget(parent)
{
  ui_.setupUi(this);

  scan_timer_ = new QTimer(this);
  connect(scan_timer_, &QTimer::timeout, this, &GridManager::scanForSources);
  scan_timer_->start(1000);
}

GridManager::~GridManager()
{

}

void GridManager::scanForSources()
{
  ros::NodeHandle nh;

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  for(const auto t: topic_info)
    if (t.datatype == "nav_msgs/OccupancyGrid" || t.datatype == "grid_map_msgs/GridMap")
      if (grids_.find(t.name) == grids_.end())
      {
        grids_[t.name] = new Grid(this, background_);
        grids_[t.name]->setTopic(t.name, t.datatype);
        grids_[t.name]->setTF2Buffer(tf_buffer_);
        grids_[t.name]->setObjectName(t.name.c_str());
        if(background_)
          grids_[t.name]->setPixelSize(background_->pixelSize());
        ui_.gridLayout->addWidget(grids_[t.name]);
      }
}

void GridManager::setTFBuffer(tf2_ros::Buffer* buffer)
{
  tf_buffer_ = buffer;
  for(auto og: grids_)
    og.second->setTF2Buffer(buffer);
}

void GridManager::updateBackground(BackgroundRaster * bg)
{
  background_ = bg;
  for(auto og: grids_)
    og.second->updateBackground(bg);
}
