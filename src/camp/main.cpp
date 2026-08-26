#include "mainwindow.h"
#include <QApplication>
#include <QString>
#include <QFileInfo>

#include "rclcpp/rclcpp.hpp"

#include "crash_handler.h"
#include "crash_log_path.h"

int main(int argc, char *argv[])
{
    // [#217] Stderr-only handlers as the first statement of main() — which is
    // as early as CAMP can reach, not "before anything at all can fault":
    // dynamic-library loading, static initialization across ~100 translation
    // units, Qt resource registration and GDAL driver registration have all
    // already run by the time control arrives here, and a fault in any of them
    // is still silent.
    // rclcpp::init() is itself a documented thrower; if it escapes,
    // std::terminate would otherwise run with the default handler and that
    // startup crash would be as silent as it was before #217. The crash-log
    // path is not resolvable yet (it comes from the ROS logging directory), so
    // this pass writes to stderr only — which the ros2 launch log captures.
    camp_crash::install_crash_handlers(-1);

    rclcpp::init(argc, argv);

    // [#217] Reinstall, now with the crash-log file.
    //
    // Ordering matters and is deliberate: after rclcpp::init() because the
    // crash-log path comes from the ROS logging directory, and before
    // QApplication so a fault during UI construction is still reported.
    // Neither claims these signals — rclcpp::init() takes only SIGINT/SIGTERM,
    // and Qt5 installs no fatal handlers — so there is nothing to conflict
    // with. An unresolvable log directory is not fatal: handlers still install
    // and write to stderr.
    camp_crash::install_crash_handlers(camp_crash::crash_log_path());

    //ros::init(argc,argv, "CCOMAutonomousMissionPlanner", ros::init_options::AnonymousName);
    QApplication a(argc, argv);
    // [#59 ADR-0003] Name the QSettings store explicitly. Without this the
    // deployed app falls back to "Unknown Organization", and — worse — splits
    // its per-layer settings and the persisted chart list across stores. These
    // names must stay stable across releases. Must be set before MainWindow,
    // which reads QSettings in its constructor.
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
        if(arg.isEmpty())  // empty arg (e.g. an empty launch substitution) -> ignore
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
