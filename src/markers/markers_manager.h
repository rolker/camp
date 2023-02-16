#ifndef MARKERS_MANAGER_H
#define MARKERS_MANAGER_H

#include <QWidget>
#include "ui_markers_manager.h"
#include <tf2_ros/transform_listener.h>

class BackgroundRaster;
class Markers;

class MarkersManager: public QWidget
{
  Q_OBJECT

public:
  explicit MarkersManager(QWidget* parent=nullptr);
  ~MarkersManager();
  void setTFBuffer(tf2_ros::Buffer *buffer);

public slots:
  void updateBackground(BackgroundRaster * bg);

private slots:
  void scanForSources();

private:
  Ui::MarkersManager ui_;

  std::map<std::string, Markers*> markers_;

  QTimer* scan_timer_;
  BackgroundRaster* background_ = nullptr;
  tf2_ros::Buffer* tf_buffer_ = nullptr;
};

#endif
