#ifndef CAMP_ROS_OBJECT_H
#define CAMP_ROS_OBJECT_H

#include <QObject>
#include "ros_client.h"

namespace camp_ros
{

class ROSObject: public ROSClient<QObject>
{
public:
  ROSObject(QObject *parent = nullptr);


};

}

#endif