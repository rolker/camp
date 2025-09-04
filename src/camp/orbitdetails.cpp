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
    auto position = orbit_->position();
    ui_.targetPositionXLineEdit->setText(QString::number(position.x()));
    ui_.targetPositionYLineEdit->setText(QString::number(position.y()));
    ui_.targetPositionZLineEdit->setText(QString::number(position.z()));
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

void OrbitDetails::on_targetPositionXLineEdit_editingFinished()
{
  positionUpdated();
}

void OrbitDetails::on_targetPositionYLineEdit_editingFinished()
{
  positionUpdated();
}

void OrbitDetails::on_targetPositionZLineEdit_editingFinished()
{
  positionUpdated();
}

void OrbitDetails::positionUpdated()
{
  if(orbit_)
  {
    auto position = orbit_->position();
    float value;
    bool ok;
    value = ui_.targetPositionXLineEdit->text().toFloat(&ok);
    if(ok)
      position.setX(value);
    value = ui_.targetPositionYLineEdit->text().toFloat(&ok);
    if(ok)
      position.setY(value);
    value = ui_.targetPositionZLineEdit->text().toFloat(&ok);
    if(ok)
      position.setZ(value);
    orbit_->setPosition(position);
  }  
}
