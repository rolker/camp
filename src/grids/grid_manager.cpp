#include "grid_manager.h"
#include "backgroundraster.h"
#include "grid.h"
#include <QTimer>

GridManager::GridManager(QWidget* parent)
  : camp_ros::ROSWidget(parent)
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
  if(node_)
  {
    auto topics = node_->get_topic_names_and_types();
    for(auto topic: topics)
    {
      for(auto topic_type: topic.second)
      {
        if(topic_type == "nav_msgs/OccupancyGrid" || topic_type == "grid_map_msgs/GridMap")
        {
          auto name = topic.first;
          if (grids_.find(name) == grids_.end())
          {
            grids_[name] = new Grid(this, background_);
            grids_[name]->nodeStarted(node_, transform_buffer_);
            grids_[name]->setTopic(name, topic_type);
            grids_[name]->setObjectName(name.c_str());
            if(background_)
              grids_[name]->setPixelSize(background_->pixelSize());
            ui_.gridLayout->addWidget(grids_[name]);
          }
        }
      }
    }
  }
}

void GridManager::onNodeUpdated()
{
  for(auto og: grids_)
    og.second->nodeStarted(node_, transform_buffer_);
}

void GridManager::updateBackground(BackgroundRaster * bg)
{
  background_ = bg;
  for(auto og: grids_)
    og.second->updateBackground(bg);
}
