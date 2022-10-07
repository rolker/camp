#ifndef ORBITDETAILS_H
#define ORBITDETAILS_H

#include <QWidget>
#include "ui_orbitdetails.h"


class Orbit;

class OrbitDetails : public QWidget
{
    Q_OBJECT

public:
    explicit OrbitDetails(QWidget *parent = 0);
    ~OrbitDetails();

    void setOrbit(Orbit* orbit);

private slots:
    void on_radiusLineEdit_editingFinished();
    void on_safetyDistanceLineEdit_editingFinished();
    void on_targetFrameLineEdit_editingFinished();

private:
    Ui::OrbitDetails ui_;
    Orbit * orbit_ = nullptr;
};

#endif
