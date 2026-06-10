#include "tracklinedetails.h"
#include "ui_tracklinedetails.h"
#include "trackline.h"

TrackLineDetails::TrackLineDetails(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TrackLineDetails)
{
    ui->setupUi(this);
}

TrackLineDetails::~TrackLineDetails()
{
    delete ui;
}

void TrackLineDetails::setTrackLine(TrackLine *trackLine)
{
    m_trackLine = trackLine;
    connect(trackLine,&TrackLine::trackLineUpdated,this,&TrackLineDetails::onTrackLineUpdated);
    onTrackLineUpdated();
}

void TrackLineDetails::onTrackLineUpdated()
{
    if(!m_trackLine)
        return;
    ui->label->setText(QString::number(m_trackLine->waypoints().length())+" waypoints");
}
