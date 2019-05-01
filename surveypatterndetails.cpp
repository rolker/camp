#include "surveypatterndetails.h"
#include "ui_surveypatterndetails.h"
#include "surveypattern.h"

SurveyPatternDetails::SurveyPatternDetails(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SurveyPatternDetails),
    updating(false)
{
    ui->setupUi(this);
    ui->alignmentComboBox->addItem("Start");
    ui->alignmentComboBox->addItem("Center");
    ui->alignmentComboBox->addItem("Finish");
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
    ui->alignmentComboBox->setAutoCompletion(surveyPattern->alignment());
    onSurveyPatternUpdated();
}

void SurveyPatternDetails::onSurveyPatternUpdated()
{
    if(!updating)
    {
        ui->lineSpacingEdit->setText(QString::number(m_surveyPattern->spacing()));
        ui->headingEdit->setText(QString::number(m_surveyPattern->direction()));
        ui->lineLengthLineEdit->setText(QString::number(m_surveyPattern->lineLength()));
        ui->totalWidthLineEdit->setText(QString::number(m_surveyPattern->totalWidth()));
    }
}

void SurveyPatternDetails::updateSurveyPattern()
{
    updating = true;
    m_surveyPattern->setDirectionAndSpacing(ui->headingEdit->text().toDouble(),ui->lineSpacingEdit->text().toDouble());
    m_surveyPattern->setLineLength(ui->lineLengthLineEdit->text().toDouble());
    m_surveyPattern->setTotalWidth(ui->totalWidthLineEdit->text().toDouble());
    switch(ui->alignmentComboBox->currentIndex())
    {
        case 0:
            m_surveyPattern->setAlignment(SurveyPattern::Alignment::start);
            break;
        case 1:
            m_surveyPattern->setAlignment(SurveyPattern::Alignment::center);
            break;
        case 2:
            m_surveyPattern->setAlignment(SurveyPattern::Alignment::finish);
            break;
    }
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

void SurveyPatternDetails::on_alignmentComboBox_activated()
{
    updateSurveyPattern();
}
