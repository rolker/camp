#ifndef GRID_MANAGER_H
#define GRID_MANAGER_H

#include "ros/ros_widget.h"
#include "ui_grid_manager.h"

class BackgroundRaster;
class Grid;

class GridManager: public camp_ros::ROSWidget
{
  Q_OBJECT

public:
  explicit GridManager(QWidget* parent=nullptr);
  ~GridManager();

public slots:
  void updateBackground(BackgroundRaster * bg);

  void onNodeUpdated() override;
private slots:
  void scanForSources();

private:
  Ui::GridManager ui_;

  std::map<std::string, Grid*> grids_;

  QTimer* scan_timer_;
  BackgroundRaster* background_ = nullptr;
};

#endif
