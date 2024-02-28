#include "ros_widget.h"

namespace camp_ros
{

ROSWidget::ROSWidget(QWidget *parent)
  :ROSClient<QWidget>(parent)
{

}



} // namespace camp_ros