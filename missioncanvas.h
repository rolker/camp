#ifndef MISSIONCANVAS_H
#define MISSIONCANVAS_H

#include <QWidget>
class QAbstractItemModel;

class MissionCanvas : public QWidget
{
    Q_OBJECT
public:
    explicit MissionCanvas(QWidget *parent = 0);

    QAbstractItemModel *model() const { return m_model;}
    void setModel(QAbstractItemModel *model);
signals:

public slots:
    //void setCurrentIndex(const QModelIndex &index);

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

private:
    QAbstractItemModel * m_model;

    double scale;
    QPointF displayCenter;

    bool isPanning;
    QPointF mouseStart;
    QPointF displayCenterStart;

};

#endif // MISSIONCANVAS_H
