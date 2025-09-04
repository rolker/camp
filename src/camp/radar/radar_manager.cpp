#include "radar_manager.h"
#include "backgroundraster.h"
#include "radar_display.h"
#include <QTimer>
#include <QColorDialog>

RadarManager::RadarManager(QWidget* parent):
  QWidget(parent)
{
  ui_.setupUi(this);

  scan_timer_ = new QTimer(this);
  connect(scan_timer_, &QTimer::timeout, this, &RadarManager::scanForSources);
  scan_timer_->start(1000);

}

RadarManager::~RadarManager()
{

}

void RadarManager::scanForSources()
{
  ros::NodeHandle nh;

  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  for(const auto t: topic_info)
    if (t.datatype == "marine_sensor_msgs/RadarSector")
      if (radar_displays_.find(t.name) == radar_displays_.end())
      {
        radar_displays_[t.name] = new RadarDisplay(this, background_);
        radar_displays_[t.name]->setTF2Buffer(tf_buffer_);
        radar_displays_[t.name]->subscribe(t.name.c_str());
        radar_displays_[t.name]->showRadar(show_radar_);
        if(background_)
          radar_displays_[t.name]->setPixelSize(background_->pixelSize());

        ui_.sourcesListWidget->addItem(t.name.c_str());
      }

  for(auto rd: radar_displays_)
    rd.second->updatePosition();
}

void RadarManager::showRadar(bool show)
{
  show_radar_ = show;
  for(auto rd: radar_displays_)
  {
    rd.second->showRadar(show);
  }
}

void RadarManager::selectRadarColor()
{
  for(auto rd: radar_displays_)
  {
    rd.second->setColor(QColorDialog::getColor(rd.second->getColor() , nullptr, "Select Color", QColorDialog::DontUseNativeDialog));
  }
}

void RadarManager::setTFBuffer(tf2_ros::Buffer* buffer)
{
  tf_buffer_ = buffer;
  for(auto rd: radar_displays_)
    rd.second->setTF2Buffer(buffer);
}

void RadarManager::updateBackground(BackgroundRaster * bg)
{
  background_ = bg;
  for(auto rd: radar_displays_)
  {
    rd.second->setParentItem(bg);
    if(bg)
      rd.second->setPixelSize(bg->pixelSize());
  }
}
