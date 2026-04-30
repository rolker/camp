// Tests for camp_ros::PlainTopicBridge, TfTopicBridge, TfDispatcher, and
// RosContext. See docs/decisions/0001-topicbridge-and-executor-contract.md.
//
// These tests pin down the threading and TF contracts:
//   - Receiver slots fire on the Qt main thread, never on the ROS executor.
//   - TF-gated bridges withhold dispatch until the target frame is reachable.
//   - TF-gated bridges drop messages whose TF never resolves (within timeout).
//   - Bridge destruction is safe and does not deadlock.
//
// The fixture spins a real rclcpp::Node + MultiThreadedExecutor on a worker
// thread and treats the test thread as the "Qt main thread."

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QObject>
#include <QSignalSpy>
#include <QThread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

#include "ros/ros_context.h"
#include "ros/tf_dispatcher.h"
#include "ros/topic_bridge.h"

using namespace std::chrono_literals;

namespace
{

// QObject receiver used to capture dispatched payloads. Records each payload
// and the thread it was delivered on, then emits gotOne() so QSignalSpy can
// pump events until dispatch completes.
class TestReceiver : public QObject
{
  Q_OBJECT
public:
  std::vector<int> int_payloads;
  std::vector<geometry_msgs::msg::PointStamped> point_payloads;
  Qt::HANDLE delivery_thread_handle = nullptr;

  void onInt(int v)
  {
    int_payloads.push_back(v);
    delivery_thread_handle = QThread::currentThreadId();
    Q_EMIT gotOne();
  }

  void onPoint(geometry_msgs::msg::PointStamped p)
  {
    point_payloads.push_back(std::move(p));
    delivery_thread_handle = QThread::currentThreadId();
    Q_EMIT gotOne();
  }

Q_SIGNALS:
  void gotOne();
};

// Pump Qt events for up to `ms` milliseconds, returning early if `predicate`
// becomes true. Used when QSignalSpy::wait() is too coarse (we want to assert
// that *no* dispatch happened within a timeout).
template<typename Pred>
bool waitForUntil(int ms, Pred predicate)
{
  QElapsedTimer timer;
  timer.start();
  while (timer.elapsed() < ms)
  {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    if (predicate()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

void pumpEventsFor(int ms)
{
  QElapsedTimer timer;
  timer.start();
  while (timer.elapsed() < ms)
  {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
    std::this_thread::sleep_for(5ms);
  }
}

geometry_msgs::msg::TransformStamped identityTransform(
  const std::string & parent, const std::string & child, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.rotation.w = 1.0;
  return t;
}

}  // namespace

class TopicBridgeTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::thread executor_thread;
  rclcpp::CallbackGroup::SharedPtr default_cbg;
  Qt::HANDLE qt_thread_handle = nullptr;

  void SetUp() override
  {
    qt_thread_handle = QThread::currentThreadId();
    static std::atomic<unsigned> global_counter{0};
    auto suffix = std::to_string(global_counter.fetch_add(1));
    node = rclcpp::Node::make_shared("camp_topic_bridge_test_" + suffix);
    buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto timer_iface = std::make_shared<tf2_ros::CreateTimerROS>(
      node->get_node_base_interface(), node->get_node_timers_interface());
    buffer->setCreateTimerInterface(timer_iface);
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*buffer, node);
    default_cbg = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor_thread = std::thread([this]() { executor->spin(); });
  }

  void TearDown() override
  {
    if (executor) executor->cancel();
    if (executor_thread.joinable()) executor_thread.join();
    executor.reset();
    tf_listener.reset();
    buffer.reset();
    node.reset();
  }
};

// ---------------------------------------------------------------------------
// PlainTopicBridge
// ---------------------------------------------------------------------------

TEST_F(TopicBridgeTest, plainBridgeReceivesPublishedMessage)
{
  TestReceiver receiver;
  camp_ros::PlainTopicBridge<std_msgs::msg::Int32, int> bridge(
    node, "/test_int", rclcpp::QoS(10), default_cbg,
    [](const std_msgs::msg::Int32 & m) -> std::optional<int> { return m.data; },
    &receiver,
    [&receiver](int v) { receiver.onInt(v); });

  // Publisher uses the same node + executor as the bridge. Wait briefly for
  // the subscription to come up before publishing.
  auto pub = node->create_publisher<std_msgs::msg::Int32>("/test_int", 10);
  waitForUntil(500, [&]() { return pub->get_subscription_count() >= 1; });

  std_msgs::msg::Int32 msg;
  msg.data = 42;
  pub->publish(msg);

  QSignalSpy spy(&receiver, &TestReceiver::gotOne);
  ASSERT_TRUE(spy.wait(2000)) << "payload was never dispatched";
  ASSERT_EQ(receiver.int_payloads.size(), 1u);
  EXPECT_EQ(receiver.int_payloads.front(), 42);
}

TEST_F(TopicBridgeTest, plainBridgeDispatchesOnQtThreadNotExecutorThread)
{
  TestReceiver receiver;
  camp_ros::PlainTopicBridge<std_msgs::msg::Int32, int> bridge(
    node, "/test_int_thread", rclcpp::QoS(10), default_cbg,
    [](const std_msgs::msg::Int32 & m) -> std::optional<int> { return m.data; },
    &receiver,
    [&receiver](int v) { receiver.onInt(v); });

  auto pub = node->create_publisher<std_msgs::msg::Int32>("/test_int_thread", 10);
  waitForUntil(500, [&]() { return pub->get_subscription_count() >= 1; });

  std_msgs::msg::Int32 msg;
  msg.data = 1;
  pub->publish(msg);

  QSignalSpy spy(&receiver, &TestReceiver::gotOne);
  ASSERT_TRUE(spy.wait(2000));
  EXPECT_EQ(receiver.delivery_thread_handle, qt_thread_handle)
    << "receiver slot must run on the Qt main thread, not the executor thread";
}

TEST_F(TopicBridgeTest, plainBridgeConverterReturningNulloptDropsMessage)
{
  TestReceiver receiver;
  camp_ros::PlainTopicBridge<std_msgs::msg::Int32, int> bridge(
    node, "/test_int_drop", rclcpp::QoS(10), default_cbg,
    [](const std_msgs::msg::Int32 & m) -> std::optional<int>
    {
      if (m.data < 0) return std::nullopt;
      return m.data;
    },
    &receiver,
    [&receiver](int v) { receiver.onInt(v); });

  auto pub = node->create_publisher<std_msgs::msg::Int32>("/test_int_drop", 10);
  waitForUntil(500, [&]() { return pub->get_subscription_count() >= 1; });

  std_msgs::msg::Int32 dropped;
  dropped.data = -7;
  pub->publish(dropped);

  std_msgs::msg::Int32 kept;
  kept.data = 13;
  pub->publish(kept);

  QSignalSpy spy(&receiver, &TestReceiver::gotOne);
  ASSERT_TRUE(spy.wait(2000));
  // Pump for a little longer to make sure no straggler arrives for the dropped one.
  pumpEventsFor(200);
  ASSERT_EQ(receiver.int_payloads.size(), 1u);
  EXPECT_EQ(receiver.int_payloads.front(), 13);
}

TEST_F(TopicBridgeTest, plainBridgeDestructIsCleanWhenIdle)
{
  TestReceiver receiver;
  {
    camp_ros::PlainTopicBridge<std_msgs::msg::Int32, int> bridge(
      node, "/test_int_destr", rclcpp::QoS(10), default_cbg,
      [](const std_msgs::msg::Int32 & m) -> std::optional<int> { return m.data; },
      &receiver,
      [&receiver](int v) { receiver.onInt(v); });
    // Bridge goes out of scope here.
  }
  // No deadlock, no crash. Pump a bit to confirm no late dispatches.
  pumpEventsFor(100);
  EXPECT_EQ(receiver.int_payloads.size(), 0u);
}

// ---------------------------------------------------------------------------
// TfTopicBridge
// ---------------------------------------------------------------------------

TEST_F(TopicBridgeTest, tfBridgeWithdrawsDispatchUntilTfArrives)
{
  TestReceiver receiver;
  camp_ros::TfTopicBridge<geometry_msgs::msg::PointStamped, geometry_msgs::msg::PointStamped> bridge(
    node, buffer, "/test_pt_late", rclcpp::QoS(10),
    /*target_frame=*/"earth",
    /*filter_queue_size=*/10,
    /*buffer_timeout=*/2s,
    default_cbg,
    [](const geometry_msgs::msg::PointStamped & m, tf2_ros::Buffer & buf)
      -> std::optional<geometry_msgs::msg::PointStamped>
    {
      // By contract, the transform is in the buffer when the converter runs.
      // We do a TimePointZero lookup with no timeout to confirm.
      auto t = buf.lookupTransform("earth", m.header.frame_id, tf2::TimePointZero);
      (void)t;
      return m;
    },
    &receiver,
    [&receiver](geometry_msgs::msg::PointStamped p) { receiver.onPoint(std::move(p)); });

  auto pub = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/test_pt_late", 10);
  waitForUntil(500, [&]() { return pub->get_subscription_count() >= 1; });

  // Publish before TF is available.
  geometry_msgs::msg::PointStamped m;
  m.header.frame_id = "base_link";
  m.header.stamp = node->now();
  m.point.x = 1.0;
  pub->publish(m);

  // No TF yet → no dispatch within a window.
  pumpEventsFor(300);
  EXPECT_EQ(receiver.point_payloads.size(), 0u)
    << "TF-gated bridge dispatched without target transform";

  // Provide TF: earth → base_link identity.
  auto tf = identityTransform("earth", "base_link", node->now());
  buffer->setTransform(tf, "test_authority", /*is_static=*/true);

  // Now the queued message should release.
  QSignalSpy spy(&receiver, &TestReceiver::gotOne);
  ASSERT_TRUE(spy.wait(2000)) << "queued message did not dispatch after TF arrived";
  ASSERT_EQ(receiver.point_payloads.size(), 1u);
  EXPECT_EQ(receiver.point_payloads.front().header.frame_id, "base_link");
}

TEST_F(TopicBridgeTest, tfBridgeDispatchesImmediatelyWhenTfPresent)
{
  // TF available BEFORE the bridge subscribes.
  auto tf = identityTransform("earth", "base_link", node->now());
  buffer->setTransform(tf, "test_authority", /*is_static=*/true);

  TestReceiver receiver;
  camp_ros::TfTopicBridge<geometry_msgs::msg::PointStamped, geometry_msgs::msg::PointStamped> bridge(
    node, buffer, "/test_pt_early", rclcpp::QoS(10),
    "earth", 10, 2s, default_cbg,
    [](const geometry_msgs::msg::PointStamped & m, tf2_ros::Buffer &)
      -> std::optional<geometry_msgs::msg::PointStamped> { return m; },
    &receiver,
    [&receiver](geometry_msgs::msg::PointStamped p) { receiver.onPoint(std::move(p)); });

  auto pub = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/test_pt_early", 10);
  waitForUntil(500, [&]() { return pub->get_subscription_count() >= 1; });

  geometry_msgs::msg::PointStamped m;
  m.header.frame_id = "base_link";
  m.header.stamp = node->now();
  m.point.y = 5.0;
  pub->publish(m);

  QSignalSpy spy(&receiver, &TestReceiver::gotOne);
  ASSERT_TRUE(spy.wait(2000));
  ASSERT_EQ(receiver.point_payloads.size(), 1u);
  EXPECT_DOUBLE_EQ(receiver.point_payloads.front().point.y, 5.0);
}

TEST_F(TopicBridgeTest, tfBridgeDropsMessageWhenTfNeverArrives)
{
  TestReceiver receiver;
  // Short buffer_timeout so the test runs fast.
  camp_ros::TfTopicBridge<geometry_msgs::msg::PointStamped, geometry_msgs::msg::PointStamped> bridge(
    node, buffer, "/test_pt_drop", rclcpp::QoS(10),
    "earth", 10, /*buffer_timeout=*/200ms, default_cbg,
    [](const geometry_msgs::msg::PointStamped & m, tf2_ros::Buffer &)
      -> std::optional<geometry_msgs::msg::PointStamped> { return m; },
    &receiver,
    [&receiver](geometry_msgs::msg::PointStamped p) { receiver.onPoint(std::move(p)); });

  auto pub = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/test_pt_drop", 10);
  waitForUntil(500, [&]() { return pub->get_subscription_count() >= 1; });

  geometry_msgs::msg::PointStamped m;
  m.header.frame_id = "no_such_frame";
  m.header.stamp = node->now();
  pub->publish(m);

  // Wait well past buffer_timeout.
  pumpEventsFor(800);
  EXPECT_EQ(receiver.point_payloads.size(), 0u)
    << "TF-gated bridge dispatched a message whose target was never reachable";
}

TEST_F(TopicBridgeTest, tfBridgeRecoversAfterTopicStopsAndResumes)
{
  auto tf = identityTransform("earth", "base_link", node->now());
  buffer->setTransform(tf, "test_authority", /*is_static=*/true);

  TestReceiver receiver;
  camp_ros::TfTopicBridge<geometry_msgs::msg::PointStamped, geometry_msgs::msg::PointStamped> bridge(
    node, buffer, "/test_pt_resume", rclcpp::QoS(10),
    "earth", 10, 2s, default_cbg,
    [](const geometry_msgs::msg::PointStamped & m, tf2_ros::Buffer &)
      -> std::optional<geometry_msgs::msg::PointStamped> { return m; },
    &receiver,
    [&receiver](geometry_msgs::msg::PointStamped p) { receiver.onPoint(std::move(p)); });

  // First publish round.
  {
    auto pub = node->create_publisher<geometry_msgs::msg::PointStamped>(
      "/test_pt_resume", 10);
    waitForUntil(500, [&]() { return pub->get_subscription_count() >= 1; });

    geometry_msgs::msg::PointStamped m;
    m.header.frame_id = "base_link";
    m.header.stamp = node->now();
    m.point.x = 1.0;
    pub->publish(m);

    QSignalSpy spy(&receiver, &TestReceiver::gotOne);
    ASSERT_TRUE(spy.wait(2000));
    // pub goes out of scope → "topic stops"
  }

  // Quiet period: no spurious dispatches.
  size_t after_first = receiver.point_payloads.size();
  pumpEventsFor(200);
  EXPECT_EQ(receiver.point_payloads.size(), after_first)
    << "bridge dispatched after publisher disappeared";

  // Resume: a fresh publisher.
  auto pub2 = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/test_pt_resume", 10);
  waitForUntil(500, [&]() { return pub2->get_subscription_count() >= 1; });

  geometry_msgs::msg::PointStamped m2;
  m2.header.frame_id = "base_link";
  m2.header.stamp = node->now();
  m2.point.x = 2.0;
  pub2->publish(m2);

  QSignalSpy spy2(&receiver, &TestReceiver::gotOne);
  ASSERT_TRUE(spy2.wait(2000)) << "bridge did not recover after topic resumed";
  ASSERT_EQ(receiver.point_payloads.size(), after_first + 1);
  EXPECT_DOUBLE_EQ(receiver.point_payloads.back().point.x, 2.0);
}

// ---------------------------------------------------------------------------
// Multiple bridges, no interference
// ---------------------------------------------------------------------------

TEST_F(TopicBridgeTest, multipleBridgesOnSameNodeDoNotInterfere)
{
  TestReceiver r1;
  TestReceiver r2;
  camp_ros::PlainTopicBridge<std_msgs::msg::Int32, int> b1(
    node, "/multi_a", rclcpp::QoS(10), default_cbg,
    [](const std_msgs::msg::Int32 & m) -> std::optional<int> { return m.data; },
    &r1, [&r1](int v) { r1.onInt(v); });
  camp_ros::PlainTopicBridge<std_msgs::msg::Int32, int> b2(
    node, "/multi_b", rclcpp::QoS(10), default_cbg,
    [](const std_msgs::msg::Int32 & m) -> std::optional<int> { return m.data * 10; },
    &r2, [&r2](int v) { r2.onInt(v); });

  auto pa = node->create_publisher<std_msgs::msg::Int32>("/multi_a", 10);
  auto pb = node->create_publisher<std_msgs::msg::Int32>("/multi_b", 10);
  waitForUntil(500, [&]() {
    return pa->get_subscription_count() >= 1 && pb->get_subscription_count() >= 1;
  });

  std_msgs::msg::Int32 ma; ma.data = 1; pa->publish(ma);
  std_msgs::msg::Int32 mb; mb.data = 2; pb->publish(mb);

  ASSERT_TRUE(waitForUntil(2000, [&]() {
    return !r1.int_payloads.empty() && !r2.int_payloads.empty();
  }));
  EXPECT_EQ(r1.int_payloads.front(), 1);
  EXPECT_EQ(r2.int_payloads.front(), 20);
}

// ---------------------------------------------------------------------------
// Callback-group isolation
// ---------------------------------------------------------------------------

TEST_F(TopicBridgeTest, sceneCallbackDoesNotStarveRealtimeCallback)
{
  // Two MutuallyExclusive callback groups can run in parallel only if the
  // executor has at least two worker threads. On a degenerate single-core
  // machine this test would deadlock; skip there.
  if (executor->get_number_of_threads() < 2)
  {
    GTEST_SKIP() << "MultiThreadedExecutor has fewer than 2 threads; "
                 << "callback-group isolation cannot be observed.";
  }

  auto realtime_cbg = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto scene_cbg = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  TestReceiver realtime_receiver;
  TestReceiver scene_receiver;

  std::atomic<bool> scene_in_callback{false};
  std::atomic<bool> release_scene{false};

  // Scene bridge: converter blocks until released, simulating a slow payload
  // conversion that would otherwise starve the realtime group on a shared
  // executor.
  camp_ros::PlainTopicBridge<std_msgs::msg::Int32, int> scene_bridge(
    node, "/iso_scene", rclcpp::QoS(10), scene_cbg,
    [&](const std_msgs::msg::Int32 & m) -> std::optional<int>
    {
      scene_in_callback.store(true);
      while (!release_scene.load()) std::this_thread::sleep_for(5ms);
      return m.data;
    },
    &scene_receiver,
    [&scene_receiver](int v) { scene_receiver.onInt(v); });

  // Realtime bridge: fast converter.
  camp_ros::PlainTopicBridge<std_msgs::msg::Int32, int> realtime_bridge(
    node, "/iso_rt", rclcpp::QoS(10), realtime_cbg,
    [](const std_msgs::msg::Int32 & m) -> std::optional<int> { return m.data; },
    &realtime_receiver,
    [&realtime_receiver](int v) { realtime_receiver.onInt(v); });

  auto scene_pub = node->create_publisher<std_msgs::msg::Int32>("/iso_scene", 10);
  auto rt_pub = node->create_publisher<std_msgs::msg::Int32>("/iso_rt", 10);
  waitForUntil(500, [&]() {
    return scene_pub->get_subscription_count() >= 1 &&
           rt_pub->get_subscription_count() >= 1;
  });

  // Block the scene group's worker.
  std_msgs::msg::Int32 sm; sm.data = 1; scene_pub->publish(sm);
  ASSERT_TRUE(waitForUntil(2000, [&]() { return scene_in_callback.load(); }))
    << "scene callback never started — cannot test isolation";

  // While scene is blocked, the realtime callback must still fire.
  std_msgs::msg::Int32 rm; rm.data = 99; rt_pub->publish(rm);
  QSignalSpy spy(&realtime_receiver, &TestReceiver::gotOne);
  ASSERT_TRUE(spy.wait(2000))
    << "realtime callback was starved by a blocked scene callback";
  EXPECT_EQ(realtime_receiver.int_payloads.front(), 99);
  EXPECT_EQ(scene_receiver.int_payloads.size(), 0u)
    << "scene receiver fired despite its converter still blocking";

  // Cleanup: release the scene callback so the bridge's subscription can
  // shut down cleanly when it goes out of scope.
  release_scene.store(true);
  pumpEventsFor(200);
}

// ---------------------------------------------------------------------------
// TfDispatcher used standalone (sub-stream feeding pattern)
// ---------------------------------------------------------------------------

TEST_F(TopicBridgeTest, tfDispatcherStandaloneDispatchesGatedOnTf)
{
  auto tf = identityTransform("earth", "base_link", node->now());
  buffer->setTransform(tf, "test_authority", /*is_static=*/true);

  TestReceiver receiver;
  camp_ros::TfDispatcher<geometry_msgs::msg::PointStamped, geometry_msgs::msg::PointStamped>
    dispatcher(
      node, buffer, "earth", 10, 2s,
      [](const geometry_msgs::msg::PointStamped & m, tf2_ros::Buffer &)
        -> std::optional<geometry_msgs::msg::PointStamped> { return m; },
      &receiver,
      [&receiver](geometry_msgs::msg::PointStamped p) { receiver.onPoint(std::move(p)); });

  geometry_msgs::msg::PointStamped m;
  m.header.frame_id = "base_link";
  m.header.stamp = node->now();
  m.point.z = 9.0;
  dispatcher.add(m);

  QSignalSpy spy(&receiver, &TestReceiver::gotOne);
  ASSERT_TRUE(spy.wait(2000));
  ASSERT_EQ(receiver.point_payloads.size(), 1u);
  EXPECT_DOUBLE_EQ(receiver.point_payloads.front().point.z, 9.0);
}

// ---------------------------------------------------------------------------
// RosContext (no fixture — avoid the executor-thread machinery; we are only
// exercising the singleton accessor and callback-group construction)
// ---------------------------------------------------------------------------

TEST(RosContextTest, singletonRoundTrip)
{
  auto node = rclcpp::Node::make_shared("camp_ros_context_test");
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  EXPECT_EQ(camp_ros::RosContext::instance(), nullptr);

  camp_ros::RosContext ctx(node, buffer);
  camp_ros::RosContext::setInstance(&ctx);
  EXPECT_EQ(camp_ros::RosContext::instance(), &ctx);

  auto realtime = ctx.group(camp_ros::RosContext::Group::Realtime);
  auto scene = ctx.group(camp_ros::RosContext::Group::Scene);
  ASSERT_NE(realtime, nullptr);
  ASSERT_NE(scene, nullptr);
  EXPECT_NE(realtime.get(), scene.get())
    << "Realtime and Scene must be distinct callback groups";

  camp_ros::RosContext::clearInstance();
  EXPECT_EQ(camp_ros::RosContext::instance(), nullptr);
}

// ---------------------------------------------------------------------------
// main: rclcpp + Qt event loop
// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QCoreApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}

#include "test_topic_bridge.moc"
