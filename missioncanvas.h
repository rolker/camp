#ifndef MISSIONCANVAS_H
#define MISSIONCANVAS_H

#include <QWidget>
#include "scaledview.h"
class QAbstractItemModel;
class QStatusBar;
class QLabel;
class BackgroundRaster;

class MissionCanvas : public QWidget
{
    Q_OBJECT
public:
    explicit MissionCanvas(QWidget *parent = 0);

    QAbstractItemModel *model() const { return m_model;}
    void setModel(QAbstractItemModel *model);
    void addWaypoint();
    void setStatusBar(QStatusBar *bar);
signals:

public slots:
    //void setCurrentIndex(const QModelIndex &index);

protected:
    void resizeEvent(QResizeEvent *event) Q_DECL_OVERRIDE;
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

private:
    enum class MouseMode {pan, addWaypoint, addTrackline};
    QAbstractItemModel * m_model;
    QStatusBar * statusBar;
    QLabel * positionLabel;
    QLabel * modeLabel;

    ScaledView view;

    MouseMode mouseMode;
    bool isPanning;
    QPointF lastMousePosition;

    BackgroundRaster * getBackgroundRaster() const;

};

#endif // MISSIONCANVAS_H
