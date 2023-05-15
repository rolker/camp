#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QGraphicsView>
#include <QGeoCoordinate>

// Widget for displaying a QGraphicsScene in a web mercator view.
// Signals are sent to report the mouse positions in WGS84 coordinates
// and to notify when the viewport changes.
// Zoom is supported by scrolling the mouse including fine zoom when
// the ctrl key is pressed.
class MapView: public QGraphicsView
{
  Q_OBJECT
public:
  explicit MapView(QWidget *parent = 0);

  // Information about the viewable portion of the scene.
  struct Viewport
  {
    // Bounds of the viewport in web mercator map units
    QRectF map_extents;

    // Size of a map unit in diplay pixels
    double pixels_per_map_unit;

    // Approximate size in meters of a map unit
    double meters_per_map_unit;
  };

signals:
  void mouseMoved(QGeoCoordinate position);
  void viewportChanged(Viewport viewport);

public slots:
  void sendViewport();

protected:
  void wheelEvent(QWheelEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void resizeEvent(QResizeEvent* event) override;
};

#endif
