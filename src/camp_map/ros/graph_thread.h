#ifndef CAMP_ROS_GRAPH_H
#define CAMP_ROS_GRAPH_H

#include <QThread>
#include "ros_common.h"
#include "rclcpp/node.hpp"

namespace camp
{
namespace ros
{

class Node;

/// Manages the ROS graph information such as nodes, topics and services.
class GraphThread: public QThread
{
  Q_OBJECT
public:
  GraphThread(Node* node);

signals:
  void updateTopic(QString name, QStringList types);
  void updateService(QString name, QStringList types);
  void addNode(QString name);
  void removeNode(QString name);

private:
  void run() override;
  bool keepRunning() const;

  /// Sleeps briefly to allow other events to be processed.
  /// This is called after emitting a number of signals to avoid flooding the event loop.
  void emitSleep();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Event::SharedPtr graph_event_;

  std::map<std::string, std::vector<std::string>> known_topics_;
  std::map<std::string, std::vector<std::string>> known_services_;
  std::map<std::string, bool> known_nodes_;

  int emit_count_ = 0;
};


}  // namespace ros
}  // namespace camp

#endif
