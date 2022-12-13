#ifndef OCCUPANCY_GRID_MANAGER_H
#define OCCUPANCY_GRID_MANAGER_H

#include <QWidget>
#include "ui_occupancy_grid_manager.h"
#include <tf2_ros/transform_listener.h>

class BackgroundRaster;
class OccupancyGrid;

class OccupancyGridManager: public QWidget
{
  Q_OBJECT

public:
  explicit OccupancyGridManager(QWidget* parent=nullptr);
  ~OccupancyGridManager();
  void setTFBuffer(tf2_ros::Buffer *buffer);

public slots:
  void updateBackground(BackgroundRaster * bg);

private slots:
  void scanForSources();

private:
  Ui::OccupancyGridManager ui_;

  std::map<std::string, OccupancyGrid*> occupancy_grids_;

  QTimer* scan_timer_;
  BackgroundRaster* background_ = nullptr;
  tf2_ros::Buffer* tf_buffer_ = nullptr;

};

#endif
