#ifndef CAMP_SPEECH_ALERTS_H
#define CAMP_SPEECH_ALERTS_H

#include <QObject>

class SpeechAlerts: public QObject
{
  Q_OBJECT
public:
  explicit SpeechAlerts(QObject *parent =0);
  ~SpeechAlerts();

signals:
  void tell(QString speech);

public slots:
  void updatePilotingMode(QString piloting_mode);

private:
  QString m_piloting_mode;
};

#endif
