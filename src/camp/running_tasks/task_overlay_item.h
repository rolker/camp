#ifndef CAMP_RUNNING_TASKS_TASK_OVERLAY_ITEM_H
#define CAMP_RUNNING_TASKS_TASK_OVERLAY_ITEM_H

#include <QObject>
#include <QGeoCoordinate>
#include <QList>
#include <QString>

#include "geographicsitem.h"

/// Renders one running task's poses on the QGraphicsScene.
///
/// Single pose  → filled circle (goto-style task).
/// Two+ poses   → polyline (survey_line, transit, etc.).
///
/// Visual states: selected (bright/bold), active (current task), done (gray),
/// pending (muted). Emits clicked(id) on mouse press for map→tree selection.
///
/// Uses geoToPixel() from GeoGraphicsItem (anchored to the Web-Mercator scene
/// origin at (0,0)); no setPos() needed — all rendering is in scene coordinates.
class TaskOverlayItem : public QObject, public GeoGraphicsItem
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public:
  explicit TaskOverlayItem(const QString& id,
                           const QList<QGeoCoordinate>& geo_poses,
                           bool is_current, bool is_done,
                           QGraphicsItem* parent = nullptr);

  int type() const override { return RunningTaskType; }

  QRectF boundingRect() const override;
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override;
  QPainterPath shape() const override;

  /// Highlight this item as the map-selected task. Named to avoid shadowing the
  /// non-virtual QGraphicsItem::setSelected (which drives scene->selectedItems()).
  void setHighlighted(bool highlighted);

signals:
  void clicked(QString id);

protected:
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;

private:
  QPainterPath buildPath() const;

  /// Scene-unit radius that renders as \p pixels display pixels at \p at,
  /// following the ADR-0003 constant-pixel-footprint convention (mirrors
  /// ShipTrack::drawTriangle's no-heading marker) so markers hold their
  /// on-screen size regardless of zoom.
  qreal sceneRadius(const QGeoCoordinate& at, qreal pixels) const;

  QString id_;
  QList<QGeoCoordinate> geo_poses_;
  bool is_current_;
  bool is_done_;
  bool is_highlighted_ = false;
};

#endif  // CAMP_RUNNING_TASKS_TASK_OVERLAY_ITEM_H
