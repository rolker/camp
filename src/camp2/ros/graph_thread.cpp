#include "graph_thread.h"
#include "node.h"

#include <QDebug>

namespace camp
{
namespace ros
{ 

GraphThread::GraphThread(Node* node):
  QThread(node),
  node_(node->node())
{
  setObjectName("ROS GraphThread");
}

void GraphThread::emitSleep()
{
  emit_count_++;
  if(emit_count_ >= 100)
  {
    emit_count_ = 0;
    msleep(50);
  }
}

void GraphThread::run()
{
  graph_event_ = node_->get_graph_event();
  while(rclcpp::ok() && !isInterruptionRequested())
  {
    node_->wait_for_graph_change(graph_event_, std::chrono::milliseconds(100));
    if(graph_event_->check_and_clear())
    {
      // Graph has changed.
      auto topics = node_->get_topic_names_and_types();

      for(const auto& topic: topics)
      {
        if(known_topics_.find(topic.first) == known_topics_.end())
        {
          // New topic
          known_topics_[topic.first] = topic.second;
          auto types = QStringList();
          for(const auto& type: topic.second)
            types.append(type.c_str());
          emit updateTopic(topic.first.c_str(), types);
          emitSleep();
        }
        else
        {
          // Existing topic, check if types changed.
          auto& known_types = known_topics_[topic.first];
          if(known_types.size() != topic.second.size() ||
             !std::equal(known_types.begin(), known_types.end(), topic.second.begin()))
          {
            known_types = topic.second;
            auto types = QStringList();
            for(const auto& type: topic.second)
              types.append(type.c_str());
            emit updateTopic(topic.first.c_str(), types);
            emitSleep();
          }
        }
      }
      // Check for removed topics.
      for(auto it = known_topics_.begin(); it != known_topics_.end(); )
      {
        if(topics.find(it->first) == topics.end())
        {
          // topic was removed
          emit updateTopic(it->first.c_str(), QStringList());
          emitSleep();
          it = known_topics_.erase(it);
        }
        else
          ++it;
      }

      auto services = node_->get_service_names_and_types();
      for(const auto& service: services)
      {
        if(known_services_.find(service.first) == known_services_.end())
        {
          // New service
          known_services_[service.first] = service.second;
          auto types = QStringList();
          for(const auto& type: service.second)
            types.append(type.c_str());
          emit updateService(service.first.c_str(), types);
          emitSleep();
        }
        else
        {
          // Existing service, check if types changed.
          auto& known_types = known_services_[service.first];
          if(known_types.size() != service.second.size() ||
             !std::equal(known_types.begin(), known_types.end(), service.second.begin()))
          {
            known_types = service.second;
            auto types = QStringList();
            for(const auto& type: service.second)
              types.append(type.c_str());
            emit updateService(service.first.c_str(), types);
            emitSleep();
          }
        }
      }
      // Check for removed services.
      for(auto it = known_services_.begin(); it != known_services_.end(); )
      {
        if(services.find(it->first) == services.end())
        {
          // service was removed
          emit updateService(it->first.c_str(), QStringList());
          emitSleep();
          it = known_services_.erase(it);
        }
        else
          ++it;
      }


      for(auto& node: known_nodes_)
        node.second = false;
      auto nodes = node_->get_node_names();
      for(const auto& node: nodes)
      {
        if(known_nodes_.find(node) == known_nodes_.end())
        {
          // New node
          known_nodes_[node] = true;
          emit addNode(node.c_str());
          emitSleep();
        }
        else
        {
          // Existing node, mark as online
          known_nodes_[node] = true;
        }
      }
      // Check for removed nodes.
      for(auto it = known_nodes_.begin(); it != known_nodes_.end(); )
      {
        if(!it->second)
        {
          // node was removed
          emit removeNode(it->first.c_str());
          emitSleep();
          it = known_nodes_.erase(it);
        }
        else
          ++it;
      }
    }
  }
}

}  // namespace ros

}  // namespace camp
