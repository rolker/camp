#ifndef CAMP_SOUND_PLAY_H
#define CAMP_SOUND_PLAY_H

#include <QWidget>
#include <memory>

namespace Ui
{
  class SoundPlay;
}

namespace sound_play
{
  class SoundClient;
}

class SoundPlay: public QWidget
{
  Q_OBJECT
public:
  explicit SoundPlay(QWidget *parent =0);
  ~SoundPlay();

signals:
  void tell(QString speech);

public slots:
  void say(QString speech);

private slots:
  void on_sayPushButton_clicked(bool checked);
  void on_whatToSayLineEdit_returnPressed();

private:
  Ui::SoundPlay* m_ui;

  std::shared_ptr<sound_play::SoundClient> m_soundClient;

};

#endif
