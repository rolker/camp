#include "camp_main_window.h"
#include <QApplication>
#include "../ros/node_manager.h"

int main(int argc, char *argv[])
{
  camp_ros::NodeManager::init(argc, argv);

  QApplication a(argc, argv);

  camp::MainWindow camp;
  camp.show();
  
  return a.exec();
}
