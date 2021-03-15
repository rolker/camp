#ifndef BACKGROUNDDETAILS_H
#define BACKGROUNDDETAILS_H

#include <QWidget>

namespace Ui {
class BackgroundDetails;
}

class BackgroundRaster;

class BackgroundDetails : public QWidget
{
    Q_OBJECT

public:
    explicit BackgroundDetails(QWidget *parent = 0);
    ~BackgroundDetails();

    void setBackgroundRaster(BackgroundRaster *bg);

private:
    Ui::BackgroundDetails *ui;
    BackgroundRaster * m_backgroundRaster;
};

#endif // BACKGROUNDDETAILS_H
