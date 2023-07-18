#include "camp_main_window.h"
#include <QLabel>
#include "../map/map.h"
#include "../map_tree_view/map_item_delegate.h"
#include "../map_tiles/map_tiles.h"
#include <QAbstractItemModelTester>
#include <QSettings>
#include "cached_file_loader.h"

namespace camp
{

MainWindow::MainWindow(QWidget *parent)
{
  QCoreApplication::setOrganizationName("UNH-CCOMJHC");
  QCoreApplication::setOrganizationDomain("ccom.unh.edu");
  QCoreApplication::setApplicationName("CCOMAutonomousMissionPlanner");

  CachedFileLoader::construct();

  ui_.setupUi(this);
  position_label_ = new QLabel(ui_.statusBar);
  ui_.statusBar->addWidget(position_label_);
  connect(ui_.mapView, &MapView::mouseMoved, this, &MainWindow::mousePositionUpdate);

  map_ = new map::Map(this);

  new QAbstractItemModelTester(map_,QAbstractItemModelTester::FailureReportingMode::Fatal, this);
  
  ui_.mapView->setMap(map_);
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
  CachedFileLoader::destruct();
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
