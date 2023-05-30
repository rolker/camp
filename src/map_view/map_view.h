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

private:
  static constexpr double min_zoom_scale_ = 1/50000.0; // 50000 m/pixel

  // Zooming in more than about 50 causes some overview tiles not in the view to suddenly
  // get paint calls, probably due to overflow in QTransform calculations. These paint
  // calls tigger tiles to load and potentially download their image, which could easily
  // go beyond a tile server's usage limits.
  static constexpr double max_zoom_scale_ = 50; // 2 cm/pixel
};

#endif
