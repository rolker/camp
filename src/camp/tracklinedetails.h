#ifndef TRACKLINEDETAILS_H
#define TRACKLINEDETAILS_H

#include <QWidget>

namespace Ui {
class TrackLineDetails;
}
class TrackLine;

class TrackLineDetails : public QWidget
{
    Q_OBJECT

public:
    explicit TrackLineDetails(QWidget *parent = 0);
    ~TrackLineDetails();

    void setTrackLine(TrackLine *trackLine);

public slots:
    void onTrackLineUpdated();

private:
    Ui::TrackLineDetails *ui;
    TrackLine *m_trackLine;
};

#endif // TRACKLINEDETAILS_H
