#ifndef CAMP_AIS_MANAGER_H
#define CAMP_AIS_MANAGER_H

#include <QWidget>
#include "ros/ros.h"
#include "marine_msgs/Contact.h"
#include "ais_contact.h"

namespace Ui
{
class AISManager;
}

class BackgroundRaster;

class AISManager: public QWidget
{
  Q_OBJECT

public:
  explicit AISManager(QWidget *parent =0);
  ~AISManager();

signals:
  void newAisReport(AISReport *report);

public slots:
  void updateBackground(BackgroundRaster * bg);
  void updateViewport(QPointF ll, QPointF ur);

private slots:
  void scanForSources();
  void addAisReport(AISReport *report);

private:
  void contactCallback(const marine_msgs::Contact::ConstPtr& message);

  Ui::AISManager* m_ui;
  std::map<std::string, ros::Subscriber> m_sources;
  QTimer* m_scan_timer;
  std::map<uint32_t, AISContact*> m_contacts;

  BackgroundRaster* m_background = nullptr;
};

#endif