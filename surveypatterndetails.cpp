#include "surveypatterndetails.h"
#include "ui_surveypatterndetails.h"
#include "surveypattern.h"

SurveyPatternDetails::SurveyPatternDetails(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SurveyPatternDetails),
    updating(false)
{
    ui->setupUi(this);
}

SurveyPatternDetails::~SurveyPatternDetails()
{
    delete ui;
}

void SurveyPatternDetails::setSurveyPattern(SurveyPattern *surveyPattern)
{
    m_surveyPattern = surveyPattern;
    connect(surveyPattern,&SurveyPattern::surveyPatternUpdated,this,&SurveyPatternDetails::onSurveyPatternUpdated);
    ui->startPoint->setWaypoint(surveyPattern->startLocationWaypoint());
    if(surveyPattern->endLocationWaypoint())
        ui->oppositePoint->setWaypoint(surveyPattern->endLocationWaypoint());
    onSurveyPatternUpdated();
}

void SurveyPatternDetails::onSurveyPatternUpdated()
{
    if(!updating)
    {
        ui->lineSpacingEdit->setText(QString::number(m_surveyPattern->spacing()));
        ui->headingEdit->setText(QString::number(m_surveyPattern->direction()));
        ui->turnArcPointCountLineEdit->setText(QString::number(m_surveyPattern->arcCount()));
        ui->lineLengthLineEdit->setText(QString::number(m_surveyPattern->lineLength()));
        ui->totalWidthLineEdit->setText(QString::number(m_surveyPattern->totalWidth()));
        ui->maxSegmentLengthLineEdit->setText(QString::number(m_surveyPattern->maxSegmentLength()));
    }
}

void SurveyPatternDetails::updateSurveyPattern()
{
    updating = true;
    m_surveyPattern->setDirectionAndSpacing(ui->headingEdit->text().toDouble(),ui->lineSpacingEdit->text().toDouble());
    m_surveyPattern->setArcCount(ui->turnArcPointCountLineEdit->text().toInt());
    m_surveyPattern->setMaxSegmentLength(ui->maxSegmentLengthLineEdit->text().toDouble());
    m_surveyPattern->setLineLength(ui->lineLengthLineEdit->text().toDouble());
    m_surveyPattern->setTotalWidth(ui->totalWidthLineEdit->text().toDouble());
    updating = false;
}

void SurveyPatternDetails::on_headingEdit_editingFinished()
{
    updateSurveyPattern();
}

void SurveyPatternDetails::on_lineSpacingEdit_editingFinished()
{
    updateSurveyPattern();
}

void SurveyPatternDetails::on_turnArcPointCountLineEdit_editingFinished()
{
    updateSurveyPattern();
}

void SurveyPatternDetails::on_lineLengthLineEdit_editingFinished()
{
    updateSurveyPattern();
}

void SurveyPatternDetails::on_totalWidthLineEdit_editingFinished()
{
    updateSurveyPattern();
}

void SurveyPatternDetails::on_maxSegmentLengthLineEdit_editingFinished()
{
    updateSurveyPattern();
}
