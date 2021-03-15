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
    ui->activeCheckBox->setChecked(m_behavior->active());
    ui->behaviorTypeComboBox->setEditText(m_behavior->behaviorType());
}

void BehaviorDetails::on_activeCheckBox_stateChanged(int state)
{
    if(m_behavior)
        m_behavior->setActive(state);
}

void BehaviorDetails::on_behaviorTypeComboBox_editTextChanged(const QString& behaviorType)
{
    if(m_behavior)
        m_behavior->setBehaviorType(behaviorType);
}
