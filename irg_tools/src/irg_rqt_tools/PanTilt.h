#ifndef irg_rqt_tools_PanTilt_h
#define irg_rqt_tools_PanTilt_h

#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>

#include <QList>
#include <QTime>
#include <QString>
#include <QWidget>

#include "ui_PanTilt.h"

namespace irg_rqt_tools {

  class PanTilt
    : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT

  public:
    PanTilt();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
                              qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
                                 const qt_gui_cpp::Settings& instance_settings);

  protected slots:
    virtual void updateTopicList();
    virtual void onPanTopicChanged(int index);
    virtual void onPanValueChanged(int value);
    virtual void onPanMinValueChanged(double value);
    virtual void onPanMaxValueChanged(double value);
    
    virtual void onTiltTopicChanged(int index);
    virtual void onTiltValueChanged(int value);
    virtual void onTiltMinValueChanged(double value);
    virtual void onTiltMaxValueChanged(double value);
    
    virtual void onPanValuePressed();
    virtual void onTiltValuePressed();
    virtual void onPanValueReleased();
    virtual void onTiltValueReleased();

  protected:
    virtual void selectTopic(const QString& topic, QComboBox* combo);

  protected:

    Ui::PanTiltWidget m_ui;
    QWidget*          m_widget;
    
    ros::Publisher    m_panPublisher;
    ros::Publisher    m_tiltPublisher;
    
    QTime             m_panValueTime;
    QTime             m_tiltValueTime;
    double            m_panValueDefault;
    double            m_tiltValueDefault;
  };
}

#endif // irg_rqt_tools_PanTilt_h
