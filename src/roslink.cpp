#include "roslink.h"
#include "ui_roslink.h"
#include "ros/node_thread.h"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <QDebug>
#include <QPainter>
#include <QGraphicsSvgItem>
#include <QColorDialog>
#include "autonomousvehicleproject.h"
#include "backgroundraster.h"
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

void ROSLink::connectROS()
{    
  camp_ros::NodeThread* node = new camp_ros::NodeThread();
  node->moveToThread(&node_thread_);
  connect(&node_thread_, &QThread::finished, node, &QObject::deleteLater);
  connect(this, &ROSLink::startNode, node, &camp_ros::NodeThread::start);
  connect(node, &camp_ros::NodeThread::started, this, &ROSLink::nodeStarted);
  connect(node, &camp_ros::NodeThread::shuttingDown, this, &ROSLink::nodeShuttingDown);


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
