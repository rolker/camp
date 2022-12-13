#include "occupancy_grid_manager.h"
#include "backgroundraster.h"
#include "occupancy_grid.h"
#include <QTimer>

OccupancyGridManager::OccupancyGridManager(QWidget* parent):
  QWidget(parent)
{
  ui_.setupUi(this);

  scan_timer_ = new QTimer(this);
  connect(scan_timer_, &QTimer::timeout, this, &OccupancyGridManager::scanForSources);
  scan_timer_->start(1000);
}

OccupancyGridManager::~OccupancyGridManager()
{

}

void OccupancyGridManager::scanForSources()
{
  ros::NodeHandle nh;

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  for(const auto t: topic_info)
    if (t.datatype == "nav_msgs/OccupancyGrid")
      if (occupancy_grids_.find(t.name) == occupancy_grids_.end())
      {
        occupancy_grids_[t.name] = new OccupancyGrid(this, background_);
        occupancy_grids_[t.name]->setTopic(t.name);
        ui_.occupancyGridLayout->addWidget(occupancy_grids_[t.name]);

      }

}

void OccupancyGridManager::setTFBuffer(tf2_ros::Buffer* buffer)
{
  tf_buffer_ = buffer;
  for(auto og: occupancy_grids_)
    og.second->setTF2Buffer(buffer);
}

void OccupancyGridManager::updateBackground(BackgroundRaster * bg)
{
  background_ = bg;
  for(auto og: occupancy_grids_)
  {
    og.second->setParentItem(bg);
    if(bg)
      og.second->setPixelSize(bg->pixelSize());
  }
}
