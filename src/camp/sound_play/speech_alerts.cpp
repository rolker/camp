#include "speech_alerts.h"
#include "sound_play_widget.h"

SpeechAlerts::SpeechAlerts(QObject* parent):
  QObject(parent)
{
    
}

SpeechAlerts::~SpeechAlerts()
{

}

void SpeechAlerts::updatePilotingMode(QString piloting_mode)
{
  if(piloting_mode != m_piloting_mode)
    emit tell(piloting_mode);
  m_piloting_mode = piloting_mode;
}


