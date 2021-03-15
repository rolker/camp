#include "mainwindow.h"
#include <QApplication>
#include <QString>
#include <QFileInfo>

#ifdef AMP_ROS
#include "ros/ros.h"
#endif

int main(int argc, char *argv[])
{
#ifdef AMP_ROS
    ros::init(argc,argv, "CCOMAutonomousMissionPlanner", ros::init_options::AnonymousName);
#endif    
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    
    
    for(int i = 1; i < argc; i++)
    {
        QString arg(argv[i]);
        if(arg.endsWith(".json", Qt::CaseInsensitive))
            w.open(arg);
        else if(QFileInfo(arg).isDir())
            w.setWorkspace(arg);
        else // try background
            w.openBackground(arg);
    }

    return a.exec();
}
