#ifndef DETAILSVIEW_H
#define DETAILSVIEW_H

#include <QWidget>

class QPushButton;
class AutonomousVehicleProject;
class BackgroundDetails;
class WaypointDetails;
class TrackLineDetails;
class SurveyPatternDetails;
class ROSDetails;
class BehaviorDetails;

class DetailsView : public QWidget
{
    Q_OBJECT
public:
    explicit DetailsView(QWidget *parent = 0);

    void setProject(AutonomousVehicleProject *project);

    QSize sizeHint() const override;

signals:
    void clearTasks();

public slots:
    void onCurrentItemChanged(const QModelIndex  &current, const QModelIndex &previous);
    void onRenamedPushButton_clicked();
    void onExecutePushButton_clicked();
    void onAppendPushButton_clicked();

private:
    AutonomousVehicleProject *m_project;

    QPushButton* m_executePushButton;
    QPushButton* m_renamePushButton;
    QPushButton* m_appendPushButton;

    QWidget * currentWidget;

    BackgroundDetails * backgroundDetails;
    WaypointDetails * waypointDetails;
    TrackLineDetails * trackLineDetails;
    SurveyPatternDetails * surveyPatternDetails;
    ROSDetails * rosDetails;
    BehaviorDetails * behaviorDetails;

    void setCurrentWidget(QWidget *widget, bool canExecute);
};

#endif // DETAILSVIEW_H
