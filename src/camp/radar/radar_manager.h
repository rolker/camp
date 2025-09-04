#ifndef CAMP_RADAR_MANAGER_H
#define CAMP_RADAR_MANAGER_H

#include <QWidget>
#include "ui_radar_manager.h"
#include <tf2_ros/transform_listener.h>

class BackgroundRaster;
class RadarDisplay;

class RadarManager: public QWidget
{
  Q_OBJECT

public:
  explicit RadarManager(QWidget *parent =0);
  ~RadarManager();
  void setTFBuffer(tf2_ros::Buffer *buffer);

public slots:
  void updateBackground(BackgroundRaster * bg);
  void showRadar(bool show);
  void selectRadarColor();

private slots:
  void scanForSources();


private:
  Ui::RadarManager ui_;

  QTimer* scan_timer_;
  BackgroundRaster* background_ = nullptr;

  std::map<std::string, RadarDisplay*> radar_displays_;

  tf2_ros::Buffer* tf_buffer_ = nullptr;

  bool show_radar_ = true;
};

#endif