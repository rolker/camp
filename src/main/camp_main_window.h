#ifndef TEST_MAP_VIEW_H
#define TEST_MAP_VIEW_H

#include <QMainWindow>
#include "ui_camp_main_window.h"

class QLabel;

namespace map
{
  class Map;
}

namespace camp
{

class MainWindow: public QMainWindow
{
  Q_OBJECT
public:
  explicit MainWindow(QWidget *parent = 0);

  void closeEvent(QCloseEvent* event) override;
  
public slots:
  void mousePositionUpdate(QGeoCoordinate position);

private:
  void readSettings();
  void writeSettings();

  Ui::MainWindow ui_;

  QLabel * position_label_;

  map::Map* map_;


};

} // namespace camp

#endif
