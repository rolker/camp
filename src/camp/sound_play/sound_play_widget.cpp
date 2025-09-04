#include "sound_play_widget.h"
#include <sound_play/sound_play.h>

SoundPlay::SoundPlay(QWidget* parent)
  :camp_ros::ROSWidget(parent)
{
  ui_.setupUi(this);
}

SoundPlay::~SoundPlay()
{

}

void SoundPlay::onNodeUpdated()
{
  if(node_)
  {
    sound_client_ = std::make_shared<sound_play::SoundClient>(node_, "project11/robotsound");
  }
  else
  {
    sound_client_.reset();
  }
}

void SoundPlay::say(QString speech)
{
  if(sound_client_)
    sound_client_->say(speech.toStdString());
}

void SoundPlay::on_sayPushButton_clicked(bool checked)
{
  say(ui_.whatToSayLineEdit->text());
}

void SoundPlay::on_whatToSayLineEdit_returnPressed()
{
  say(ui_.whatToSayLineEdit->text());
}
