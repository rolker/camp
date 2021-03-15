#ifndef PLATFORMDETAIL_H
#define PLATFORMDETAIL_H

#include <QWidget>

namespace Ui {
class PlatformDetails;
}

class Platform;

class PlatformDetails : public QWidget
{
    Q_OBJECT

public:
    explicit PlatformDetails(QWidget *parent = 0);
    ~PlatformDetails();

    void setPlatform(Platform *platform);

private slots:
    void on_speedLineEdit_editingFinished();

private:
    Ui::PlatformDetails *ui;
    Platform *m_platform;
};

#endif // PLATFORMDETAIL_H
