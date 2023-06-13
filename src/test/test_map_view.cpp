#include <QApplication>
#include "test_map_view.h"
#include <QLabel>
#include "../map/map.h"
#include "../map_tree_view/map_item_delegate.h"
#include "../map_tiles/map_tiles.h"
#include <QAbstractItemModelTester>
#include "../ros/node_manager.h"


TestMapView::TestMapView(QWidget *parent)
{
  ui_.setupUi(this);
  position_label_ = new QLabel(ui_.statusBar);
  ui_.statusBar->addWidget(position_label_);
  connect(ui_.mapView, &MapView::mouseMoved, this, &TestMapView::mousePositionUpdate);

  map_ = new map::Map(this);

  new QAbstractItemModelTester(map_,QAbstractItemModelTester::FailureReportingMode::Fatal, this);
  
  ui_.mapView->setMap(map_);
  ui_.mapTreeView->setMap(map_);
}

void TestMapView::mousePositionUpdate(QGeoCoordinate position)
{
  QString posText = position.toString(QGeoCoordinate::Degrees) + " (" + position.toString(QGeoCoordinate::DegreesMinutesWithHemisphere) + ")";
  position_label_->setText(posText);
}

int main(int argc, char *argv[])
{
  camp_ros::NodeManager::init(argc, argv);

  QApplication a(argc, argv);

  TestMapView tmv;
  tmv.show();
  
  return a.exec();
}
