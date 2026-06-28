#include "roslink.h"
#include "ui_roslink.h"
#include "ros/node_thread.h"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <QDebug>
#include <QPainter>
#include <QGraphicsSvgItem>
#include <QColorDialog>
#include <QApplication>
#include "autonomousvehicleproject.h"
#include <QTimer>
#include <tf2/utils.h>
//#include "geographic_msgs/GeoPoint.h"


ROSLink::ROSLink(QWidget* parent): QWidget(parent), m_ui(new Ui::ROSLink)
{
    m_ui->setupUi(this);

  qRegisterMetaType<rclcpp::Node::SharedPtr>();
  qRegisterMetaType<tf2_ros::Buffer::SharedPtr>();
    
    qRegisterMetaType<QGeoCoordinate>();
    //connectROS();
    
    // m_watchdog_timer = new QTimer(this);
    // connect(m_watchdog_timer, SIGNAL(timeout()), this, SLOT(watchdogUpdate()));
}

ROSLink::~ROSLink()
{
  // [#59] Stop the ROS spin thread before node_thread_ (a QThread member) is
  // destroyed. node_thread_ runs an event loop (QThread::exec) that outlives
  // NodeThread::start returning, so it is still running at teardown; destroying
  // a running QThread aborts. Guarded so it is a no-op if ROS was never connected.
  if(node_thread_.isRunning())
  {
    // rclcpp::shutdown() FIRST: NodeThread::start blocks in executor.spin(),
    // which only returns once rclcpp is shut down. On the SIGINT path rclcpp's
    // signal handler already did this; on the normal window-close path nothing
    // else does, so without it spin never returns, quit()'s posted event is
    // never processed, and wait() deadlocks. Idempotent, so it's safe on both
    // paths. Mirrors camp_map's Node::~Node. quit() then exits the thread's exec()
    // loop and wait() joins it (firing QThread::finished -> NodeThread::deleteLater).
    rclcpp::shutdown();
    node_thread_.quit();
    node_thread_.wait();
  }
}

void ROSLink::connectROS()
{
  camp_ros::NodeThread* node = new camp_ros::NodeThread();
  node->moveToThread(&node_thread_);
  connect(&node_thread_, &QThread::finished, node, &QObject::deleteLater);
  connect(this, &ROSLink::startNode, node, &camp_ros::NodeThread::start);
  connect(node, &camp_ros::NodeThread::started, this, &ROSLink::nodeStarted);
  connect(node, &camp_ros::NodeThread::shuttingDown, this, &ROSLink::nodeShuttingDown);
  // [#59] Quit the Qt app when the ROS node shuts down (e.g. SIGINT/Ctrl-C from
  // the terminal) so the GUI exits cleanly instead of lingering. The retired
  // camp2 sandbox wired the same; the deployed app did not — which, once the
  // shutdown-time null-node crash was fixed (ROSClient::nodeStarted guard), left
  // Ctrl-C hanging with the window still open.
  connect(node, &camp_ros::NodeThread::shuttingDown, qApp, &QCoreApplication::quit);


  node_thread_.start();
  emit startNode();

}

void ROSLink::nodeStarted(rclcpp::Node::SharedPtr node, tf2_ros::Buffer::SharedPtr buffer)
{
  node_ = node;
  transform_buffer_ = buffer;
  emit rosConnected(node, buffer);
}

void ROSLink::nodeShuttingDown()
{
  node_.reset();
  transform_buffer_.reset();
  emit rosConnected(node_, transform_buffer_);
}

rclcpp::Node::SharedPtr ROSLink::node()
{
  return node_;
}

tf2_ros::Buffer::SharedPtr ROSLink::tfBuffer()
{
  return transform_buffer_;
}
