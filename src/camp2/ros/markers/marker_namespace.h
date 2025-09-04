#ifndef CAMP_ROS_MARKERS_MARKER_NAMESPACE_H
#define CAMP_ROS_MARKERS_MARKER_NAMESPACE_H

#include "../layer.h"

namespace camp_ros
{

class Marker;
class MarkerData;

class MarkerNamespace: public Layer
{
  Q_OBJECT
  Q_INTERFACES(QGraphicsItem)
public:
  MarkerNamespace(MapItem* parent, NodeManager* node_manager, QString marker_namespace);

  enum { Type =  map::MarkerNamespaceType };

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }


public slots:
  void updateMarker(const MarkerData& data);

private:
  std::map<uint32_t, Marker*> markers() const;  

};

}

#endif
