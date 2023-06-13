#ifndef CAMP_ROS_MARKERS_MARKER_H
#define CAMP_ROS_MARKERS_MARKER_H

#include "../layer.h"
#include "markers.h"

namespace camp_ros
{

class Marker: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  Marker(MapItem* parent, NodeManager* node_manager, uint32_t id);

  enum { Type =  map::MarkerType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  uint32_t id() const;

public slots:
  void updateMarker(const MarkerData& data);

private slots:
  void checkExpired();

private:
  uint32_t id_ = 0;
  MarkerData data_;
  bool expired_ = false;

};

}

#endif
