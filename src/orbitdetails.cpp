#include "orbitdetails.h"
#include "orbit.h"

OrbitDetails::OrbitDetails(QWidget *parent) :
    QWidget(parent)
{
  ui_.setupUi(this);
}

OrbitDetails::~OrbitDetails()
{

}

void OrbitDetails::setOrbit(Orbit* orbit)
{
  orbit_ = orbit;
  if(orbit_)
  {
    ui_.radiusLineEdit->setText(QString::number(orbit_->radius()));
    ui_.safetyDistanceLineEdit->setText(QString::number(orbit_->safetyDistance()));
    ui_.targetFrameLineEdit->setText(orbit_->targetFrame().c_str());
  }
}

void OrbitDetails::on_radiusLineEdit_editingFinished()
{
  if(orbit_)
  {
    bool ok;
    auto r = ui_.radiusLineEdit->text().toDouble(&ok);
    if(ok)
      orbit_->setRadius(r);
  }
}

void OrbitDetails::on_safetyDistanceLineEdit_editingFinished()
{
  if(orbit_)
  {
    bool ok;
    auto d = ui_.safetyDistanceLineEdit->text().toDouble(&ok);
    if(ok)
      orbit_->setSafetyDistance(d);
  }
}

void OrbitDetails::on_targetFrameLineEdit_editingFinished()
{
  if(orbit_)
  {
    orbit_->setTargetFrame(ui_.targetFrameLineEdit->text().toStdString());
  }
}
