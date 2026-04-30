#ifndef CAMP_ROS_TF_DISPATCHER_H
#define CAMP_ROS_TF_DISPATCHER_H

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <QObject>
#include <QPointer>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>

namespace camp_ros
{

/// TF-gated dispatcher for stamped messages of type MsgT.
///
/// Wraps tf2_ros::MessageFilter to delay messages until the requested target
/// frame is reachable. When the filter dispatches a message (on the ROS
/// executor thread), TfDispatcher invokes the converter to produce a
/// value-typed PayloadT, then posts the payload to a Qt receiver via
/// Qt::QueuedConnection.
///
/// The converter runs on the executor thread and may use the TF buffer with
/// tf2::TimePointZero (no timeout) — by contract, the transform is already
/// in the buffer when the converter is called.
///
/// TfDispatcher does not own a subscription. Callers feed it messages via
/// add(). For the common "subscribe + gate" case, see TopicBridge.
///
/// Lifetime contract: the receiver QObject must outlive this dispatcher.
/// In practice, hold the dispatcher as a member of the receiver.
template<typename MsgT, typename PayloadT>
class TfDispatcher
{
public:
  using MsgConstSharedPtr = std::shared_ptr<const MsgT>;
  using Converter = std::function<std::optional<PayloadT>(const MsgT&, tf2_ros::Buffer&)>;
  using ReceiverFn = std::function<void(PayloadT)>;

  TfDispatcher(rclcpp::Node::SharedPtr node,
               tf2_ros::Buffer::SharedPtr buffer,
               const std::string & target_frame,
               uint32_t queue_size,
               std::chrono::nanoseconds buffer_timeout,
               Converter converter,
               QObject * receiver,
               ReceiverFn receiver_fn)
  : node_(std::move(node)),
    buffer_(std::move(buffer))
  {
    filter_ = std::make_shared<tf2_ros::MessageFilter<MsgT>>(
      *buffer_,
      target_frame,
      queue_size,
      node_->get_node_logging_interface(),
      node_->get_node_clock_interface(),
      buffer_timeout);

    // The lambda captures everything by value so it is safe for
    // tf2_ros::MessageFilter to hold even if the dispatcher is destroyed
    // (the filter and its callbacks tear down together).
    filter_->registerCallback(
      [buffer = buffer_, converter = std::move(converter),
       receiver_ptr = QPointer<QObject>(receiver),
       receiver_fn = std::move(receiver_fn)]
      (const MsgConstSharedPtr & msg)
      {
        if (!receiver_ptr) return;
        auto payload = converter(*msg, *buffer);
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
      });
  }

  ~TfDispatcher() = default;

  TfDispatcher(const TfDispatcher &) = delete;
  TfDispatcher & operator=(const TfDispatcher &) = delete;

  /// Feed a message into the filter. Will dispatch (eventually) when the
  /// target transform is available, or be dropped after buffer_timeout.
  void add(const MsgConstSharedPtr & msg) { filter_->add(msg); }
  void add(const MsgT & msg) { filter_->add(std::make_shared<MsgT>(msg)); }

  /// Connect a message_filters::SimpleFilter (e.g. message_filters::Subscriber)
  /// to feed this dispatcher.
  template<typename FilterT>
  void connectInput(FilterT & upstream) { filter_->connectInput(upstream); }

  /// Underlying tf2_ros::MessageFilter. Exposed for advanced use.
  std::shared_ptr<tf2_ros::MessageFilter<MsgT>> filter() const { return filter_; }

private:
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer::SharedPtr buffer_;
  std::shared_ptr<tf2_ros::MessageFilter<MsgT>> filter_;
};

} // namespace camp_ros

#endif
