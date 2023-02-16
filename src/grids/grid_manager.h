#ifndef GRID_MANAGER_H
#define GRID_MANAGER_H

#include <QWidget>
#include "ui_grid_manager.h"
#include <tf2_ros/transform_listener.h>

class BackgroundRaster;
class Grid;

class GridManager: public QWidget
{
  Q_OBJECT

public:
  explicit GridManager(QWidget* parent=nullptr);
  ~GridManager();
  void setTFBuffer(tf2_ros::Buffer *buffer);

public slots:
  void updateBackground(BackgroundRaster * bg);

private slots:
  void scanForSources();

private:
  Ui::GridManager ui_;

  std::map<std::string, Grid*> grids_;

  QTimer* scan_timer_;
  BackgroundRaster* background_ = nullptr;
  tf2_ros::Buffer* tf_buffer_ = nullptr;

};

#endif
