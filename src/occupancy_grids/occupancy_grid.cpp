#include "occupancy_grid.h"
#include <tf2_ros/transform_listener.h>
#include <QDebug>

OccupancyGrid::OccupancyGrid(QWidget* parent, QGraphicsItem *parentItem):
    QWidget(parent),
    GeoGraphicsItem(parentItem)
{
  ui_.setupUi(this);

}

QRectF OccupancyGrid::boundingRect() const
{
  return QRectF();
}

void OccupancyGrid::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{

}

void OccupancyGrid::setTopic(std::string topic)
{
  if(!spinner_)
  {
    spinner_ = std::make_shared<ros::AsyncSpinner>(1, &ros_queue_);
      spinner_->start();
  }
  ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::OccupancyGrid>(topic, 1, boost::bind(&OccupancyGrid::dataCallback, this, _1), ros::VoidPtr(), &ros_queue_);
  subscriber_ = ros::NodeHandle().subscribe(ops);

  ui_.topicLabel->setText(topic.c_str());
}

void OccupancyGrid::setTF2Buffer(tf2_ros::Buffer* buffer)
{
  tf_buffer_ = buffer;
}

void OccupancyGrid::setPixelSize(double s)
{
  pixel_size_ = s;
}

void OccupancyGrid::dataCallback(const nav_msgs::OccupancyGrid::ConstPtr &data)
{
  qDebug() << "occupancy grid data!";
  qDebug() << data->info.height << " x " << data->info.width << " at " << data->info.resolution << " meters resolution";
}

void OccupancyGrid::newGridAvailable()
{
  if(current_grid_)
    prepareGeometryChange();

  {
    std::lock_guard<std::mutex> lock(new_grid_mutex_);
    current_grid_ = new_grid_;
    new_grid_.reset();
  }

  GeoGraphicsItem::update();
}
