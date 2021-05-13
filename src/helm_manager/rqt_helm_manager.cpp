#include "rqt_helm_manager.h"
#include "ui_rqt_helm_manager.h"
#include <pluginlib/class_list_macros.h>

namespace camp
{

HelmManagerPlugin::HelmManagerPlugin():rqt_gui_cpp::Plugin(), m_ui(nullptr), m_widget(nullptr)
{
  setObjectName("HelmManager");
}

void HelmManagerPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  m_widget = new QWidget();
  m_ui = new Ui::HelmManagerPlugin;
  m_ui->setupUi(m_widget);

  connect(m_ui->robotNamespaceLineEdit, &QLineEdit::editingFinished, this, &HelmManagerPlugin::on_robotNamespaceLineEdit_editingFinished);
  
  m_widget->setWindowTitle(m_widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  
  context.addWidget(m_widget);

}

void HelmManagerPlugin::shutdownPlugin()
{

}

void HelmManagerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    QString robotNamespace = m_ui->robotNamespaceLineEdit->text();
    instance_settings.setValue("robotNamespace", robotNamespace);
}

void HelmManagerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    QString robotNamespace = instance_settings.value("robotNamespace", "").toString();
    m_ui->robotNamespaceLineEdit->setText(robotNamespace);
    m_ui->helmManager->updateRobotNamespace(robotNamespace);
}

void HelmManagerPlugin::on_robotNamespaceLineEdit_editingFinished()
{
  ROS_INFO_STREAM(m_ui->robotNamespaceLineEdit->text().toStdString());
  m_ui->helmManager->updateRobotNamespace(m_ui->robotNamespaceLineEdit->text());
}

} // namespace camp

PLUGINLIB_EXPORT_CLASS(camp::HelmManagerPlugin, rqt_gui_cpp::Plugin)
