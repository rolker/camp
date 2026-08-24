#ifndef CAMP_ROS_GRIDS_GRID_MAP_H
#define CAMP_ROS_GRIDS_GRID_MAP_H

#include "../layer.h"
#include "grid_map_msgs/msg/grid_map.hpp"
#include <QtConcurrent>
#include <QMutex>
#include <QImage>
#include <QPointF>
#include <string>
#include <utility>
#include <vector>

namespace camp
{
namespace ros
{
namespace grids
{

struct GridMapLayerData
{
  QImage grid_image;
  std::string layer_name;
  std::pair<double, double> range;
};

struct GridMapData
{
  std::vector<GridMapLayerData> layers;
  QPointF center;
  float meters_per_pixel = 1.0;
};

class GridLayer;

class GridMap: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)

public:
  GridMap(MapItem* parent, Node* node, QString topic);
  ~GridMap() override;   // joins the in-flight render worker before teardown

  /// [camp#63 / camp#141] Select the colour ramp for this layer by marine_colormap
  /// palette name; persists and re-renders the last received grid. Unknown name ->
  /// grayscale.
  void setColormap(const std::string& name);

protected:
  /// [camp#208] Render only while the layer is actually shown, and catch the
  /// hidden->shown transition. A GridMap dataset (an S-57 chart layer, say) is
  /// latched: it arrives once and is never republished, so unlike a costmap this
  /// layer cannot rely on the next message to repaint it. Skipping work while
  /// hidden therefore REQUIRES re-rendering on the transition, or an unchecked
  /// layer would come back permanently blank.
  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

  void contextMenu(QMenu* menu) override;
  void readSettings() override;
  void writeSettings() override;

signals:
  void newGridData(GridMapData data);

private:
  void gridMapCallback(const grid_map_msgs::msg::GridMap &data);

  // Worker body (runs on a QtConcurrent thread). Takes its inputs by value so
  // it never touches mutex_-guarded state: the message and colormap name are
  // snapshotted under the lock at launch.
  void processGridMap(grid_map_msgs::msg::GridMap data, std::string colormap_name);
  // Renders `data` with the `colormap_name` palette into `out`; returns false (no
  // emit) when the message can't be converted, has no layers, or has no transform.
  bool renderToData(const grid_map_msgs::msg::GridMap &data, const std::string &colormap_name,
                    GridMapData &out);

  // Render-scheduling helpers. *Locked variants assume mutex_ is held. Only one
  // worker runs at a time; a request that arrives mid-render sets render_pending_
  // and is coalesced into a single follow-up render when the worker finishes —
  // so a colormap change (or newer message) during a render is never lost.
  void requestRenderLocked();
  void startRenderLocked();
  void onProcessFinished();   // worker thread, on each render's completion

  GridLayer * gridLayer(const QString & layer_name) const;

private slots:
  void updateGrid(const GridMapData& data);
  void updateGridLayer(const GridMapLayerData& data);

private:
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
  std::string topic_;

  // Guards every member below — they are touched from the ROS callback thread
  // (gridMapCallback), the UI thread (setColormap / readSettings), and the
  // QtConcurrent worker (onProcessFinished).
  QMutex mutex_;
  QFuture<void> process_future_;
  bool rendering_ = false;       // a worker is active
  bool render_pending_ = false;  // a request arrived while rendering_; render once more
  bool shutdown_ = false;        // set by the dtor so the worker stops relaunching

  // [camp#63 / camp#141] marine_colormap palette applied to the normalised grid
  // values. Default grayscale (the camp_map post-#59 default); selectable per layer.
  std::string colormap_name_{"grayscale"};

  // Last received message, cached so a colormap change can re-render without
  // waiting for the next publish (live costmaps would also pick it up).
  grid_map_msgs::msg::GridMap last_msg_;
  bool has_last_msg_ = false;

};

} // namespace grids
} // namespace ros
} // namespace camp

Q_DECLARE_METATYPE(camp::ros::grids::GridMapLayerData);
Q_DECLARE_METATYPE(camp::ros::grids::GridMapData);

#endif
