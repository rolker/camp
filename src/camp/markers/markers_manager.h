#ifndef MARKERS_MANAGER_H
#define MARKERS_MANAGER_H

#include "ros/ros_widget.h"
#include "ui_markers_manager.h"

class BackgroundRaster;
class Markers;

class MarkersManager: public camp_ros::ROSWidget
{
  Q_OBJECT

public:
  explicit MarkersManager(QWidget* parent=nullptr);
  ~MarkersManager();

public slots:
  void updateBackground(BackgroundRaster * bg);

private slots:
  void scanForSources();

private:
  Ui::MarkersManager ui_;

  std::map<std::string, Markers*> markers_;

  QTimer* scan_timer_;
  BackgroundRaster* background_ = nullptr;
};

#endif
