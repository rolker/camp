#include "map_view.h"
#include <QWheelEvent>
#include "web_mercator.h"
#include <QAbstractSlider>
#include <QScrollBar>
#include <QGuiApplication>

MapView::MapView(QWidget *parent) : QGraphicsView(parent)
{
  connect(horizontalScrollBar(), &QAbstractSlider::valueChanged, this, &MapView::sendViewport);
  connect(verticalScrollBar(), &QAbstractSlider::valueChanged, this , &MapView::sendViewport);
}

void MapView::wheelEvent(QWheelEvent *event)
{
  // wheel turn angles are encoded in 1/8 degree increments.
  auto angle_delta_degrees = event->angleDelta().y()/8.0;
  
  double zoom_level_per_degree = 0.02;
  
  // fine zoom if ctrl key is pressed
  if(QGuiApplication::keyboardModifiers().testFlag(Qt::ControlModifier))
    zoom_level_per_degree /= 3.0;

  double scale_change = pow(2,zoom_level_per_degree*angle_delta_degrees);

  scale(scale_change, scale_change);

  auto new_scale = transform().m11();

  if(new_scale < 1.0)
    scale(1/new_scale, 1/new_scale);
  else if (new_scale > 33554432) // zoom level 25, 2^25
    scale(33554432/new_scale, 33554432/new_scale);
    
  new_scale = transform().m11();

  event->accept();
  sendViewport();
}

void MapView::mouseMoveEvent(QMouseEvent *event)
{
  QGraphicsView::mouseMoveEvent(event);
  QPointF transformedMouse = mapToScene(event->pos());
  emit mouseMoved(web_mercator::mapToGeo(transformedMouse));
}

void MapView::resizeEvent(QResizeEvent* event)
{
  QGraphicsView::resizeEvent(event);
  sendViewport();
}

void MapView::sendViewport()
{
  Viewport viewport;
  QPointF top_left = mapToScene(frameRect().topLeft());
  QPointF bottom_right = mapToScene(frameRect().bottomRight());
  viewport.map_extents = QRectF(top_left, bottom_right);
  viewport.meters_per_map_unit = web_mercator::metersPerPixel(viewport.map_extents.center());
  if (viewport.map_extents.width() > 0.0)
    viewport.pixels_per_map_unit = frameRect().width()/viewport.map_extents.width();
  else
    viewport.pixels_per_map_unit = 0.0;
  emit viewportChanged(viewport);
}

