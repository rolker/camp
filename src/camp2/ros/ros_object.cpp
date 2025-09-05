#include "ros_object.h"

namespace camp
{
namespace ros
{

ROSObject::ROSObject(QObject *parent)
  :ROSClient<QObject>(parent)
{

}

} // namespace ros
} // namespace camp
