#include "mainwindow.h"
#include <QApplication>
#include <QString>
#include <QFileInfo>

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc,argv, "CCOMAutonomousMissionPlanner", ros::init_options::AnonymousName);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    
    
    for(int i = 1; i < argc; i++)
    {
        QString arg(argv[i]);
        if(arg.endsWith(".json", Qt::CaseInsensitive))
            //w.open(arg);
            QMetaObject::invokeMethod(&w, "open", Qt::QueuedConnection, Q_ARG(QString, arg));
        else if(QFileInfo(arg).isDir())
            w.setWorkspace(arg);
        else // try background
            //w.openBackground(arg);
            QMetaObject::invokeMethod(&w, "openBackground", Qt::QueuedConnection, Q_ARG(QString, arg));
    }

    return a.exec();
}
