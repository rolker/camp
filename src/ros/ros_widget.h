#ifndef CAMP_ROS_WIDGET_H
#define CAMP_ROS_WIDGET_H

#include <QWidget>
#include "ros_client.h"

namespace camp_ros
{

class ROSWidget: public ROSClient<QWidget>
{
public:
  ROSWidget(QWidget *parent = nullptr);


};

}

#endif