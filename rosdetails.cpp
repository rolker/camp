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
