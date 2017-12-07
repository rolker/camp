#include "rosnodedetails.h"
#include "ui_rosnodedetails.h"
#include "rosnode.h"
#include <QDebug>

ROSNodeDetails::ROSNodeDetails(QWidget* parent) :
    QWidget(parent),
    ui(new Ui::ROSNodeDetails)
{
    ui->setupUi(this);
}

ROSNodeDetails::~ROSNodeDetails()
{
    delete ui;
}

void ROSNodeDetails::setROSNode(ROSNode* rosNode)
{
    m_rosNode = rosNode;
    Qt::CheckState s = Qt::Unchecked;
    if(rosNode->active())
        s = Qt::Checked;
    ui->activeCheckBox->setCheckState(s);
}

void ROSNodeDetails::on_activeCheckBox_stateChanged(int state)
{
    qDebug() << "ROSNodeDetails active: " << state;
    m_rosNode->setActive(state);
}


