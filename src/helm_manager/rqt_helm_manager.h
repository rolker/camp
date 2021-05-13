#ifndef CAMP_RQT_HELM_MANAGER_H
#define CAMP_RQT_HELM_MANAGER_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

namespace Ui
{
class HelmManagerPlugin;
}


namespace camp {

class HelmManagerPlugin: public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
  HelmManagerPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);


private:
  void on_robotNamespaceLineEdit_editingFinished();

  Ui::HelmManagerPlugin* m_ui;
  QWidget* m_widget;

};

} // namespace camp

#endif
