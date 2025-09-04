#ifndef CAMP_SOUND_PLAY_H
#define CAMP_SOUND_PLAY_H

#include "ros/ros_widget.h"
#include "ui_sound_play_widget.h"

namespace sound_play
{
  class SoundClient;
}

class SoundPlay: public camp_ros::ROSWidget
{
  Q_OBJECT
public:
  explicit SoundPlay(QWidget *parent =0);
  ~SoundPlay();

  void onNodeUpdated() override;

public slots:
  void say(QString speech);

private slots:
  void on_sayPushButton_clicked(bool checked);
  void on_whatToSayLineEdit_returnPressed();

private:
  Ui::SoundPlay ui_;

  std::shared_ptr<sound_play::SoundClient> sound_client_;

};

#endif
