#include "backgrounddetails.h"
#include "ui_backgrounddetails.h"
#include "backgroundraster.h"

BackgroundDetails::BackgroundDetails(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BackgroundDetails)
{
    ui->setupUi(this);
}

BackgroundDetails::~BackgroundDetails()
{
    delete ui;
}

void BackgroundDetails::setBackgroundRaster(BackgroundRaster *bg)
{
    m_backgroundRaster = bg;
    ui->pathLineEdit->setText(bg->filename());
    ui->projectionPlainTextEdit->setPlainText(bg->projection());
}
