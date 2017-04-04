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
    onSurveyPatternUpdated();
}

void SurveyPatternDetails::onSurveyPatternUpdated()
{
    if(!updating)
    {
        ui->lineSpacingEdit->setText(QString::number(m_surveyPattern->spacing()));
        ui->headingEdit->setText(QString::number(m_surveyPattern->direction()));
    }
}

void SurveyPatternDetails::updateSurveyPattern()
{
    updating = true;
    m_surveyPattern->setDirectionAndSpacing(ui->headingEdit->text().toDouble(),ui->lineSpacingEdit->text().toDouble());
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
