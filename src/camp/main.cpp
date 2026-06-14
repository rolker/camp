#include "mainwindow.h"
#include <QApplication>
#include <QString>
#include <QFileInfo>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //ros::init(argc,argv, "CCOMAutonomousMissionPlanner", ros::init_options::AnonymousName);
    QApplication a(argc, argv);
    // [#59 ADR-0003] Name the QSettings store explicitly (matches the camp2
    // sandbox, main_window.cpp). Without this the deployed app falls back to
    // "Unknown Organization", and — worse — splits its per-layer settings and
    // the persisted chart list into a different store than camp2 writes. Must be
    // set before MainWindow, which reads QSettings in its constructor.
    QCoreApplication::setOrganizationName("UNH-CCOMJHC");
    QCoreApplication::setOrganizationDomain("ccom.unh.edu");
    QCoreApplication::setApplicationName("CCOMAutonomousMissionPlanner");
    MainWindow w;
    w.setStyleSheet("QSplitter::handle{background: #8080A0;}");
    w.show();

    auto args = rclcpp::remove_ros_arguments(argc, argv);    
    
    for(std::size_t i = 1; i < args.size(); i++)
    {
        QString arg(args[i].c_str());
        if(arg.isEmpty())  // empty arg (e.g. an empty background_chart) -> ignore
            continue;
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
