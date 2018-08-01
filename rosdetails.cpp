#include "rosdetails.h"
#include "ui_rosdetails.h"
#include "roslink.h"
#include <QDebug>

ROSDetails::ROSDetails(QWidget* parent) :
    QWidget(parent),
    ui(new Ui::ROSDetails)
{
    ui->setupUi(this);
}

ROSDetails::~ROSDetails()
{
    delete ui;
}

void ROSDetails::setROSLink(ROSLink* rosLink)
{
    m_rosLink = rosLink;
    rosLink->setROSDetails(this);
    Qt::CheckState s = Qt::Unchecked;
    if(rosLink->active())
        s = Qt::Checked;
    ui->activeCheckBox->setCheckState(s);
}

void ROSDetails::on_activeCheckBox_stateChanged(int state)
{
    qDebug() << "ROSDetails active: " << state;
    m_rosLink->setActive(state);
}

void ROSDetails::on_standbyPushButton_clicked(bool checked)
{
    qDebug() << "ROSDetails helm mode: standby";
    m_rosLink->setHelmMode("standby");
}

void ROSDetails::on_surveyPushButton_clicked(bool checked)
{
    qDebug() << "ROSDetails helm mode: survey";
    m_rosLink->setHelmMode("survey");
}

void ROSDetails::on_loiterPushButton_clicked(bool checked)
{
    qDebug() << "ROSDetails helm mode: loiter";
    m_rosLink->setHelmMode("loiter");
}

void ROSDetails::updateVehicleStatus(const QString& status)
{
    ui->vehicleStatusTextBrowser->setText(status);
}

void ROSDetails::on_sendWaypointIndexPushButton_clicked(bool checked)
{
    qDebug() << "send current index: " << ui->sendWaypointSpinBox->value();
    m_rosLink->sendWaypointIndexUpdate(ui->sendWaypointSpinBox->value());
}


void ROSDetails::heartbeatDelay(double seconds)
{
    QPalette pal = palette();
    if(seconds < 2.0)
        pal.setColor(QPalette::Background, Qt::green);
    else if (seconds < 5.0)
        pal.setColor(QPalette::Background, Qt::yellow);
    else
        pal.setColor(QPalette::Background, Qt::red);
    this->setAutoFillBackground(true);
    this->setPalette(pal);
}

void ROSDetails::rangeAndBearingUpdate(double range, ros::Time const & range_timestamp, double bearing, ros::Time const & bearing_timestamp)
{
    QString rblabel = "Range: " + QString::number(int(range)) + " m, Bearing: " + QString::number(int(bearing)) + " degs";
    ui->rangeBearingLineEdit->setText(rblabel);

    ros::Time now = ros::Time::now();
    QPalette pal = palette();
    if(now-range_timestamp < ros::Duration(5) && now-bearing_timestamp < ros::Duration(5))
    {
        pal.setColor(QPalette::Foreground, Qt::black);
        pal.setColor(QPalette::Background, Qt::white);
    }
    else
    {
        pal.setColor(QPalette::Foreground, Qt::darkGray);
        pal.setColor(QPalette::Background, Qt::yellow);
    }
    ui->rangeBearingLineEdit->setPalette(pal);
}

void ROSDetails::sogUpdate(qreal sog, qreal sog_avg)
{
    QString sogLabel = "SOG: " + QString::number(sog,'f',1) + ", avg: " + QString::number(sog_avg,'f',1);
    ui->sogLineEdit->setText(sogLabel);
}
