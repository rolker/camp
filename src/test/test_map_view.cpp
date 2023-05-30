#include <QApplication>
#include "test_map_view.h"
#include <QLabel>
#include "../map/map.h"
#include "../map/map_item_delegate.h"
#include "../map_tiles/map_tiles.h"
#include <QAbstractItemModelTester>


TestMapView::TestMapView(QWidget *parent)
{
  ui_.setupUi(this);
  position_label_ = new QLabel(ui_.statusBar);
  ui_.statusBar->addWidget(position_label_);
  connect(ui_.mapView, &MapView::mouseMoved, this, &TestMapView::mousePositionUpdate);

  map_ = new map::Map(this);

  new QAbstractItemModelTester(map_,QAbstractItemModelTester::FailureReportingMode::Fatal, this);
  
  ui_.mapView->setScene(map_->scene());
  connect(ui_.mapView, &MapView::viewportChanged, map_, &map::Map::viewportChanged);

  ui_.mapTreeView->setDragEnabled(true);
  ui_.mapTreeView->viewport()->setAcceptDrops(true);
  ui_.mapTreeView->setDropIndicatorShown(true);
  ui_.mapTreeView->setItemDelegate( new map::MapItemDelegate(this));
  ui_.mapTreeView->setModel(map_);
}

void TestMapView::mousePositionUpdate(QGeoCoordinate position)
{
  QString posText = position.toString(QGeoCoordinate::Degrees) + " (" + position.toString(QGeoCoordinate::DegreesMinutesWithHemisphere) + ")";
  position_label_->setText(posText);
}

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);

  TestMapView tmv;
  tmv.show();
  
  return a.exec();
}
