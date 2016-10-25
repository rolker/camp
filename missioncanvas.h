#ifndef MISSIONCANVAS_H
#define MISSIONCANVAS_H

#include <QWidget>

class MissionCanvas : public QWidget
{
    Q_OBJECT
public:
    explicit MissionCanvas(QWidget *parent = 0);

    void setBackgroundImage(QImage &bgimage);
signals:

public slots:

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

private:
    double scale;
    QPointF backgroundDisplayCenter;

    bool isPanning;
    QPointF mouseStart;
    QPointF backgroundDisplayCenterStart;

    typedef std::map<int,QPixmap> Mipmaps;
    Mipmaps backgroundImages;
};

#endif // MISSIONCANVAS_H
