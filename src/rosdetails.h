#ifndef ROSNODEDETAILS_H
#define ROSNODEDETAILS_H

#include <QWidget>

namespace Ui
{
class ROSDetails;
}

namespace ros
{
    class Time;
}

class ROSLink;

class ROSDetails : public QWidget
{
    Q_OBJECT
    
public:
    explicit ROSDetails(QWidget *parent =0);
    ~ROSDetails();
    
    void setROSLink(ROSLink *rosLink);
    
public slots:
    void heartbeatDelay(double seconds, ros::Time const & last_heartbeat_timestamp, ros::Time const & last_heartbeat_receive_time);
    void rangeAndBearingUpdate(double range, ros::Time const &range_timestamp, double bearing, ros::Time const &bearing_timestamp);
    void sogUpdate(qreal sog, qreal sog_avg);
    void updateHelmMode(QString const &helm_mode);
    void sendNextItem();
    void restartMission();

private slots:
    void on_standbyPushButton_clicked(bool checked);
    void on_autonomousPushButton_clicked(bool checked);

    void on_stopPingingPushButton_clicked(bool checked);
    void on_startPingingPushButton_clicked(bool checked);
    void on_pingAndLogPushButton_clicked(bool checked);

    void updateVehicleStatus(QString const &status);
    void updateMissionStatus(QString const &status);
    void on_gotoLinePushButton_clicked(bool checked);
    void on_startLinePushButton_clicked(bool checked);
    
    void on_missionStatusTextBrowser_customContextMenuRequested(const QPoint &pos);
    
private:
    Ui::ROSDetails* ui;
#ifdef AMP_ROS
    ROSLink *m_rosLink;
#endif
};

#endif // ROSNODEDETAILS_H
