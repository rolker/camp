#ifndef ROSNODEDETAILS_H
#define ROSNODEDETAILS_H

#include <QWidget>

namespace Ui
{
class ROSNodeDetails;
}

class ROSNode;

class ROSNodeDetails : public QWidget
{
    Q_OBJECT
    
public:
    explicit ROSNodeDetails(QWidget *parent =0);
    ~ROSNodeDetails();
    
    void setROSNode(ROSNode *rosNode);

private slots:
    void on_activeCheckBox_stateChanged(int state);
    
private:
    Ui::ROSNodeDetails* ui;
    ROSNode *m_rosNode;
};

#endif // ROSNODEDETAILS_H
