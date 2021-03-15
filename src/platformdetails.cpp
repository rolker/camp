#include "platformdetails.h"
#include "ui_platformdetails.h"
#include "platform.h"

PlatformDetails::PlatformDetails(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlatformDetails)
{
    ui->setupUi(this);
}

PlatformDetails::~PlatformDetails()
{
    delete ui;
}

void PlatformDetails::setPlatform(Platform *platform)
{
    m_platform = platform;
    ui->speedLineEdit->setText(QString::number(platform->speed()));
}

void PlatformDetails::on_speedLineEdit_editingFinished()
{
    m_platform->setSpeed(ui->speedLineEdit->text().toDouble());
}
