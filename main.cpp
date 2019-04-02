#include "mainwindow.h"
#include <QApplication>

#ifdef AMP_ROS
#include "ros/ros.h"
#endif

int main(int argc, char *argv[])
{
#ifdef AMP_ROS
    ros::init(argc,argv, "AutonomousMissionPlanner", ros::init_options::AnonymousName);
#endif    
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    
    if(argc > 1)
        w.open(argv[1]);

    return a.exec();
}
