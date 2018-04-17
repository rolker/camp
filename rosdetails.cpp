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

