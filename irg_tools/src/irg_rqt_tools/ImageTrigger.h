#ifndef irg_rqt_tools_ImageTrigger_h
#define irg_rqt_tools_ImageTrigger_h

#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>

#include <ui_ImageTrigger.h>

#include <QList>
#include <QTime>
#include <QString>
#include <QWidget>

namespace irg_rqt_tools {

  class ImageTrigger
    : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT

  public:
    ImageTrigger();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
                              qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
                                 const qt_gui_cpp::Settings& instance_settings);
    
  public slots:
    void refreshTopics();

  protected:
    QStringList getTriggerTopics();    

  protected:

    Ui::ImageTriggerPanel  m_ui;
    QWidget*               m_widget;
  };

} // namespace irg_rqt_tools

#endif // irg_rqt_tools_ImageTrigger_h
