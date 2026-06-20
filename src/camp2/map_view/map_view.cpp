#include "map_view.h"
#include <QWheelEvent>
#include "web_mercator.h"
#include <QAbstractSlider>
#include <QScrollBar>
#include <QGuiApplication>
#include <QOpenGLWidget>
#include "../map/map.h"
#include <QSettings>

namespace camp
{

const double MapView::min_zoom_scale_;
const double MapView::max_zoom_scale_;

MapView::MapView(QWidget *parent) : QGraphicsView(parent)
{
  // [camp#90 / I4] Use a QOpenGLWidget as the viewport so map layers can issue
  // native OpenGL inside their paint() (QPainter::beginNativePainting), e.g. the
  // GPU display-time warp of GGGS raster tiles. The existing CPU layers
  // (RasterLayer pixmaps, MapTiles, markers) keep working: Qt's GL paint engine
  // rasterizes their QPainter calls onto the same context, and they remain
  // pixel-registered because the scene→viewport transform is unchanged. Slice-1
  // risk to watch: this is an app-wide viewport swap (verify CPU-layer
  // registration; watch the camp#98 zoom/pan memory path).
  setViewport(new QOpenGLWidget(this));
  setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  connect(horizontalScrollBar(), &QAbstractSlider::valueChanged, this, &MapView::sendViewport);
  connect(verticalScrollBar(), &QAbstractSlider::valueChanged, this , &MapView::sendViewport);
  scale(min_zoom_scale_, -min_zoom_scale_);
}

void MapView::setMap(map::Map* map)
{
  setScene(map->scene());
  connect(this, &MapView::viewportChanged, map, &map::Map::viewportChanged);
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

  auto start_scale = transform().m11();

  auto new_scale = std::max(min_zoom_scale_, std::min(max_zoom_scale_, start_scale*scale_change));

  scale_change = new_scale/start_scale;

  scale(scale_change, scale_change);
    
  event->accept();
  update();
  //sendViewport();
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
  viewport.meters_per_map_unit = web_mercator::metersPerUnit(viewport.map_extents.center());
  if (viewport.map_extents.width() > 0.0)
    viewport.pixels_per_map_unit = frameRect().width()/viewport.map_extents.width();
  else
    viewport.pixels_per_map_unit = 0.0;
  emit viewportChanged(viewport);
}

void MapView::readSettings()
{
  QSettings settings;

  settings.beginGroup("MapView");
  settings.beginGroup(objectName());
  qreal new_scale = settings.value("scale", min_zoom_scale_).toReal();
  auto scale_change = new_scale/transform().m11();
  scale(scale_change, scale_change);
  centerOn(settings.value("center").toPointF());
  settings.endGroup();
  settings.endGroup();
}

void MapView::writeSettings()
{
  QSettings settings;

  settings.beginGroup("MapView");
  settings.beginGroup(objectName());
  settings.setValue("scale", transform().m11());
  settings.setValue("center", mapToScene(frameRect().center()));
  settings.endGroup();
  settings.endGroup();

}

}  // namespace camp
