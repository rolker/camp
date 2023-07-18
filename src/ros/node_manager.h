#ifndef CAMP_ROS_NODE_MANAGER_H
#define CAMP_ROS_NODE_MANAGER_H

#include "../tools/layer_manager.h"
#include <QThread>
#include <tf2_ros/transform_listener.h>

namespace tools
{
  class ToolsManager;
}

namespace camp_ros
{

// Monitors status of the ROS core and starts/stop the ROS Node.
class NodeManager: public tools::LayerManager
{
  Q_OBJECT
public:
  NodeManager(tools::ToolsManager* tools_manager);
  ~NodeManager();

  enum { Type = map::NodeManagerType};

  int type() const override
  {
    // Enable the use of qgraphicsitem_cast with this item.
    return Type;
  }

  // This static method should be called before the QApplication is 
  // created. This calls ros::init which can modify argv and argc which are also
  // passed to QApplication. QApplication docs state that argv and argc should
  // not be changed once the application is started.
  static void init(int& argc, char **argv);

  tf2_ros::Buffer& transformBuffer();

public slots:
  void nodeStarted();
  void nodeShuttingDown();

signals:
  void startNode(NodeManager* node_manager);

  // Signal emitted with the current list of avaiable topics.
  // Topics are listed in a map of topic name as keys and type as values.
  void topicsAvailable(QMap<QString, QString> topics);

  void shuttingDownRos();

private slots:
  void checkRosCore();

private:
  QThread node_thread_;

  tf2_ros::Buffer transform_buffer_;

};

} // namespace camp_ros

#endif
