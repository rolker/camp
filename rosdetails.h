#ifndef ROSNODEDETAILS_H
#define ROSNODEDETAILS_H

#include <QWidget>

namespace Ui
{
class ROSDetails;
}

class ROSLink;

class ROSDetails : public QWidget
{
    Q_OBJECT
    
public:
    explicit ROSDetails(QWidget *parent =0);
    ~ROSDetails();
    
    void setROSLink(ROSLink *rosLink);

private slots:
    void on_activeCheckBox_stateChanged(int state);
    void on_standbyPushButton_clicked(bool checked);
    void on_surveyPushButton_clicked(bool checked);
    void on_loiterPushButton_clicked(bool checked);
    
private:
    Ui::ROSDetails* ui;
    ROSLink *m_rosLink;
};

#endif // ROSNODEDETAILS_H
