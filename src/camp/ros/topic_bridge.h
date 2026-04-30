#ifndef CAMP_ROS_TOPIC_BRIDGE_H
#define CAMP_ROS_TOPIC_BRIDGE_H

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <QObject>
#include <QPointer>

#include <message_filters/subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "tf_dispatcher.h"

namespace camp_ros
{

/// Plain (non-TF-gated) topic bridge.
///
/// Owns an rclcpp::Subscription. Each received message is passed through the
/// converter on the ROS executor thread; if the converter returns a payload,
/// it is dispatched to the receiver via Qt::QueuedConnection.
///
/// Lifetime contract: the receiver QObject must outlive this bridge. In
/// practice, hold the bridge as a member of the receiver.
template<typename MsgT, typename PayloadT>
class PlainTopicBridge
{
public:
  using Converter = std::function<std::optional<PayloadT>(const MsgT &)>;
  using ReceiverFn = std::function<void(PayloadT)>;

  PlainTopicBridge(rclcpp::Node::SharedPtr node,
                   const std::string & topic,
                   const rclcpp::QoS & qos,
                   rclcpp::CallbackGroup::SharedPtr callback_group,
                   Converter converter,
                   QObject * receiver,
                   ReceiverFn receiver_fn)
  : node_(std::move(node)),
    topic_(topic)
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = std::move(callback_group);

    subscription_ = node_->create_subscription<MsgT>(
      topic, qos,
      [converter = std::move(converter),
       receiver_ptr = QPointer<QObject>(receiver),
       receiver_fn = std::move(receiver_fn)]
      (const std::shared_ptr<const MsgT> msg)
      {
        if (!receiver_ptr) return;
        auto payload = converter(*msg);
        if (!payload) return;
        QObject * raw_receiver = receiver_ptr.data();
        if (!raw_receiver) return;
        QMetaObject::invokeMethod(
          raw_receiver,
          [receiver_ptr, receiver_fn, value = std::move(*payload)]() mutable
          {
            if (receiver_ptr) receiver_fn(std::move(value));
          },
          Qt::QueuedConnection);
      },
      options);
  }

  ~PlainTopicBridge() = default;

  PlainTopicBridge(const PlainTopicBridge &) = delete;
  PlainTopicBridge & operator=(const PlainTopicBridge &) = delete;

  std::string topic() const { return topic_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  typename rclcpp::Subscription<MsgT>::SharedPtr subscription_;
};

/// TF-gated topic bridge.
///
/// Owns a message_filters::Subscriber connected to a TfDispatcher. Messages
/// are released only when the target frame is reachable; the converter runs
/// on the executor thread with a TF buffer guaranteed-available, and the
/// payload is dispatched to the receiver via Qt::QueuedConnection.
template<typename MsgT, typename PayloadT>
class TfTopicBridge
{
public:
  using Converter = typename TfDispatcher<MsgT, PayloadT>::Converter;
  using ReceiverFn = typename TfDispatcher<MsgT, PayloadT>::ReceiverFn;

  TfTopicBridge(rclcpp::Node::SharedPtr node,
                tf2_ros::Buffer::SharedPtr buffer,
                const std::string & topic,
                const rclcpp::QoS & qos,
                const std::string & target_frame,
                uint32_t filter_queue_size,
                std::chrono::nanoseconds buffer_timeout,
                rclcpp::CallbackGroup::SharedPtr callback_group,
                Converter converter,
                QObject * receiver,
                ReceiverFn receiver_fn)
  : node_(std::move(node)),
    topic_(topic)
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = std::move(callback_group);
    subscriber_.subscribe(node_, topic, qos.get_rmw_qos_profile(), options);

    dispatcher_ = std::make_unique<TfDispatcher<MsgT, PayloadT>>(
      node_, std::move(buffer), target_frame,
      filter_queue_size, buffer_timeout,
      std::move(converter), receiver, std::move(receiver_fn));
    dispatcher_->connectInput(subscriber_);
  }

  ~TfTopicBridge() = default;

  TfTopicBridge(const TfTopicBridge &) = delete;
  TfTopicBridge & operator=(const TfTopicBridge &) = delete;

  std::string topic() const { return topic_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  // dispatcher_ owns the MessageFilter that subscriber_ feeds via
  // connectInput(). Declare dispatcher_ before subscriber_ so that on
  // destruction subscriber_ is torn down (and stops feeding the filter)
  // before the filter itself is destroyed. Otherwise an in-flight ROS
  // callback can call MessageFilter::add() on freed memory.
  std::unique_ptr<TfDispatcher<MsgT, PayloadT>> dispatcher_;
  message_filters::Subscriber<MsgT> subscriber_;
};

} // namespace camp_ros

#endif
