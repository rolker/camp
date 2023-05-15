#include <QApplication>
#include "test_map_view.h"
#include <QLabel>
#include "../map_tiles/map_tiles.h"

TestMapView::TestMapView(QWidget *parent)
{
  ui_.setupUi(this);
  position_label_ = new QLabel(ui_.statusBar);
  ui_.statusBar->addWidget(position_label_);
  connect(ui_.mapView, &MapView::mouseMoved, this, &TestMapView::mousePositionUpdate);

  map_tiles::MapTiles* openstreetmap = new map_tiles::MapTiles();
  connect(ui_.mapView, &MapView::viewportChanged, openstreetmap, &map_tiles::MapTiles::updateViewport);

  openstreetmap->setLabel("openstreetmap");
  openstreetmap->setBaseUrl("https://tile.openstreetmap.org/");

  scene_.addItem(openstreetmap);

  map_tiles::MapTiles* openseamap = new map_tiles::MapTiles();
  openseamap->setLabel("openseamap");
  openseamap->setBaseUrl("https://tiles.openseamap.org/seamark/");
  connect(ui_.mapView, &MapView::viewportChanged, openseamap, &map_tiles::MapTiles::updateViewport);

  scene_.addItem(openseamap);

  ui_.mapView->setScene(&scene_);
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
