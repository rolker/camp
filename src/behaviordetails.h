#ifndef BEHAVIORDETAILS_H
#define BEHAVIORDETAILS_H

#include <QWidget>

namespace Ui
{
class BehaviorDetails;
}

class Behavior;

class BehaviorDetails : public QWidget
{
    Q_OBJECT
    
public:
    explicit BehaviorDetails(QWidget *parent= nullptr);
    ~BehaviorDetails();
    
    void setBehavior(Behavior *behavior);
    
private slots:
    void on_activeCheckBox_stateChanged(int state);
    void on_behaviorTypeComboBox_editTextChanged(QString const &behaviorType);

private:
    Ui::BehaviorDetails* ui;
    Behavior *m_behavior;
};

#endif // BEHAVIORDETAILS_H
