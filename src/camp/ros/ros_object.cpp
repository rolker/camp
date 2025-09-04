#include "ros_object.h"

namespace camp_ros
{

ROSObject::ROSObject(QObject *parent)
  :ROSClient<QObject>(parent)
{

}



} // namespace camp_ros