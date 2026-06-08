#include "main_window.h"
#include <QApplication>
#include <QLabel>
#include "../map/map.h"
#include "../map_tree_view/map_item_delegate.h"
#include "../map_tiles/map_tiles.h"
#include "../ros/node.h"
#include <QAbstractItemModelTester>
#include <QSettings>

namespace camp
{

MainWindow::MainWindow(QWidget *parent)
  :QMainWindow(parent)
{
  QCoreApplication::setOrganizationName("UNH-CCOMJHC");
  QCoreApplication::setOrganizationDomain("ccom.unh.edu");
  QCoreApplication::setApplicationName("CCOMAutonomousMissionPlanner");

  ui_.setupUi(this);
  position_label_ = new QLabel(ui_.statusBar);
  ui_.statusBar->addWidget(position_label_);
  connect(ui_.mapView, &MapView::mouseMoved, this, &MainWindow::mousePositionUpdate);

  map_ = new map::Map(this);

  // Attach the ROS node to the map's tools manager. Done here in the app layer
  // (rather than inside Map) so the map core library stays ROS-free.
  auto ros_node = new ros::Node(map_->toolsManager());
  connect(ros_node, &ros::Node::shuttingDownRos, qApp, &QCoreApplication::quit);

  new QAbstractItemModelTester(map_,QAbstractItemModelTester::FailureReportingMode::Fatal, this);
  
  ui_.mapView->setMap(map_);
  ui_.mapView2->setMap(map_);
  ui_.mapTreeView->setMap(map_);

  readSettings();
}

void MainWindow::mousePositionUpdate(QGeoCoordinate position)
{
  QString posText = position.toString(QGeoCoordinate::Degrees) + " (" + position.toString(QGeoCoordinate::DegreesMinutesWithHemisphere) + ")";
  position_label_->setText(posText);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  writeSettings();
  QMainWindow::closeEvent(event);
}

void MainWindow::readSettings()
{
  QSettings settings;

  settings.beginGroup("MainWindow");
  resize(settings.value("size", QSize(1024, 768)).toSize());
  move(settings.value("pos", QPoint(200, 200)).toPoint());
  ui_.splitter->restoreState(settings.value("splitterSizes").toByteArray());
  settings.endGroup();

  ui_.mapView->readSettings();
}

void MainWindow::writeSettings()
{
  QSettings settings;

  settings.beginGroup("MainWindow");
  settings.setValue("size", size());
  settings.setValue("pos", pos());
  settings.setValue("splitterSizes", ui_.splitter->saveState());
  settings.endGroup();

  ui_.mapView->writeSettings();
}

}
