#include "sound_play_widget.h"
#include "ui_sound_play_widget.h"
#include <sound_play/sound_play.h>
#include "ros/ros.h"
#include <QDebug>

SoundPlay::SoundPlay(QWidget* parent):
  QWidget(parent),
  m_ui(new Ui::SoundPlay)
{
  m_ui->setupUi(this);
  ros::NodeHandle nh;
  m_soundClient = std::shared_ptr<sound_play::SoundClient>(new sound_play::SoundClient(nh,"project11/robotsound"));
}

SoundPlay::~SoundPlay()
{

}

void SoundPlay::say(QString speech)
{
  qDebug() << "say: " << speech;
  m_soundClient->say(speech.toStdString());
}

void SoundPlay::on_sayPushButton_clicked(bool checked)
{
  say(m_ui->whatToSayLineEdit->text());
}

void SoundPlay::on_whatToSayLineEdit_returnPressed()
{
  say(m_ui->whatToSayLineEdit->text());
}
