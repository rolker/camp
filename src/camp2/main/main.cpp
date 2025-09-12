#include "main_window.h"
#include <QApplication>
#include "../ros/node.h"

int main(int argc, char *argv[])
{
  // Let ROS modify argc and argv as needed before sending to QApplication.
  camp::ros::Node::init(argc, argv);

  // Arguments should not be changed after this point.
  QApplication a(argc, argv);
  qSetMessagePattern("[%{time hh:mm:ss.zzz}] %{message}");
  
  camp::MainWindow camp;
  camp.show();
  
  return a.exec();
}
