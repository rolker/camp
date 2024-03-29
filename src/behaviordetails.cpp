#include "behaviordetails.h"
#include "ui_behaviordetails.h"
#include "behavior.h"

BehaviorDetails::BehaviorDetails(QWidget* parent): QWidget(parent),ui(new Ui::BehaviorDetails),m_behavior(nullptr)
{
    ui->setupUi(this);
}

BehaviorDetails::~BehaviorDetails()
{
    delete ui;
}

void BehaviorDetails::setBehavior(Behavior* behavior)
{
    m_behavior = behavior;
    ui->enabledCheckBox->setChecked(m_behavior->enabled());
    ui->behaviorTypeComboBox->setEditText(m_behavior->behaviorType());
    ui->behaviorDataPlainTextEdit->setPlainText(m_behavior->behaviorData());
}

void BehaviorDetails::on_enabledCheckBox_stateChanged(int state)
{
    if(m_behavior)
        m_behavior->setEnabled(state);
}

void BehaviorDetails::on_behaviorTypeComboBox_editTextChanged(const QString& behaviorType)
{
    if(m_behavior)
        m_behavior->setBehaviorType(behaviorType);
}

void BehaviorDetails::on_behaviorDataPlainTextEdit_textChanged()
{
    if(m_behavior)
        m_behavior->setBehaviorData(ui->behaviorDataPlainTextEdit->toPlainText());
}

