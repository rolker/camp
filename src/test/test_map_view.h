#ifndef TEST_MAP_VIEW_H
#define TEST_MAP_VIEW_H

#include <QMainWindow>
#include "ui_test_map_view.h"

class QLabel;

namespace map
{
  class Map;
}

class TestMapView: public QMainWindow
{
  Q_OBJECT
public:
  explicit TestMapView(QWidget *parent = 0);

public slots:
  void mousePositionUpdate(QGeoCoordinate position);

private:
  Ui::TestMapView ui_;

  QLabel * position_label_;

  map::Map* map_;
};

#endif
