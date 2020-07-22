#include "PanTilt.h"

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <std_msgs/Float64.h>

#include <QComboBox>
#include <QPushButton>
#include <QStringList>

#include "Utils.h"

namespace irg_rqt_tools {

  PanTilt::PanTilt()
    : 
    rqt_gui_cpp::Plugin(),
    m_widget(0)
  {
    setObjectName("PanTilt");
    m_panValueTime.start();
    m_panValueDefault = 0.0;
    m_tiltValueTime.start();
    m_tiltValueDefault = 0.0;
  }

  void PanTilt::initPlugin(qt_gui_cpp::PluginContext& context)
  {
     m_widget = new QWidget();
     m_ui.setupUi(m_widget);

    int serialNum = context.serialNumber();
    if( serialNum > 1) {
      m_widget->setWindowTitle(m_widget->windowTitle() + " (" + QString::number(serialNum) + ")");
    }
    context.addWidget(m_widget);

    updateTopicList();
    
    m_ui.panTopicCombo->setCurrentIndex(m_ui.panTopicCombo->findText(""));
    connect(m_ui.panTopicCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(onPanTopicChanged(int)));

    m_ui.tiltTopicCombo->setCurrentIndex(m_ui.tiltTopicCombo->findText(""));
    connect(m_ui.tiltTopicCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(onTiltTopicChanged(int)));

    m_ui.panRefreshButton->setIcon(QIcon::fromTheme("view-refresh"));
    connect(m_ui.panRefreshButton, SIGNAL(pressed()), this, SLOT(updateTopicList()));
    m_ui.tiltRefreshButton->setIcon(QIcon::fromTheme("view-refresh"));
    connect(m_ui.tiltRefreshButton, SIGNAL(pressed()), this, SLOT(updateTopicList()));

    connect(m_ui.panSlider,  SIGNAL(valueChanged(int)), this, SLOT(onPanValueChanged(int)));
    connect(m_ui.tiltSlider, SIGNAL(valueChanged(int)), this, SLOT(onTiltValueChanged(int)));

    connect(m_ui.panMinSpin, SIGNAL(valueChanged(double)), this, SLOT(onPanMinValueChanged(double)));
    connect(m_ui.panMaxSpin, SIGNAL(valueChanged(double)), this, SLOT(onPanMaxValueChanged(double)));
    connect(m_ui.tiltMinSpin, SIGNAL(valueChanged(double)), this, SLOT(onTiltMinValueChanged(double)));
    connect(m_ui.tiltMaxSpin, SIGNAL(valueChanged(double)), this, SLOT(onTiltMaxValueChanged(double)));

    connect(m_ui.panValueBut, SIGNAL(pressed()),  this, SLOT(onPanValuePressed()));
    connect(m_ui.tiltValueBut, SIGNAL(pressed()), this, SLOT(onTiltValuePressed()));
    connect(m_ui.panValueBut, SIGNAL(released()),  this, SLOT(onPanValueReleased()));
    connect(m_ui.tiltValueBut, SIGNAL(released()), this, SLOT(onTiltValueReleased()));
  }

  void PanTilt::shutdownPlugin()
  {
    m_panPublisher.shutdown();
    m_tiltPublisher.shutdown();
  }

  void PanTilt::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
  {
    QString panTopic  = m_ui.panTopicCombo->currentText();
    double  panMin    = m_ui.panMinSpin->value();
    double  panMax    = m_ui.panMaxSpin->value();
    QString tiltTopic = m_ui.tiltTopicCombo->currentText();
    double  tiltMin    = m_ui.tiltMinSpin->value();
    double  tiltMax    = m_ui.tiltMaxSpin->value();
    
    instance_settings.setValue("panTopic",  panTopic);
    instance_settings.setValue("panMin",    panMin);
    instance_settings.setValue("panMax",    panMax);
    instance_settings.setValue("panDefault", m_panValueDefault);
    instance_settings.setValue("tiltTopic", tiltTopic);
    instance_settings.setValue("tiltMin",   tiltMin);
    instance_settings.setValue("tiltMax",   tiltMax);
    instance_settings.setValue("tiltDefault", m_tiltValueDefault);
    
  }

  // we say 3 seconds here even though the timer is set to 2 seconds... 
  static void setDefaultTooltip(double defaultVal, QPushButton* button) {
    QString tt;
    tt.sprintf("Press to set default value (%.2f).\nTo change default value, hold button for at least 3 seconds", defaultVal);
    button->setToolTip(tt);
  }
  
  void PanTilt::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
  {
    QString panTopic  = instance_settings.value("panTopic", "").toString();
    QString tiltTopic = instance_settings.value("tiltTopic", "").toString();
    selectTopic(panTopic, m_ui.panTopicCombo);
    selectTopic(tiltTopic, m_ui.tiltTopicCombo);
    double panMin  = instance_settings.value("panMin",  -3.14).toDouble();
    double panMax  = instance_settings.value("panMax",   3.14).toDouble();
    double tiltMin = instance_settings.value("tiltMin", -1.57).toDouble();
    double tiltMax = instance_settings.value("tiltMax",  1.57).toDouble();
    m_ui.panMinSpin->setValue(panMin);
    m_ui.panMaxSpin->setValue(panMax);
    m_ui.tiltMinSpin->setValue(tiltMin);
    m_ui.tiltMaxSpin->setValue(tiltMax);
    
    m_panValueDefault  = instance_settings.value("panDefault", 0.0).toDouble();
    m_tiltValueDefault = instance_settings.value("tiltDefault", 0.0).toDouble();
    setDefaultTooltip(m_panValueDefault, m_ui.panValueBut);
    setDefaultTooltip(m_tiltValueDefault, m_ui.tiltValueBut);
    int panValue = (int)(100*m_panValueDefault);
    int tiltValue = (int)(100*m_tiltValueDefault);
    m_ui.panSlider->setValue(panValue);
    m_ui.tiltSlider->setValue(tiltValue);
    
    onPanValueChanged(panValue);
    onTiltValueChanged(tiltValue);
  }

  void PanTilt::updateTopicList()
  {
    // save current topics
    QString panTopic  = m_ui.panTopicCombo->currentText();
    QString tiltTopic = m_ui.tiltTopicCombo->currentText();

    m_ui.panTopicCombo->clear();
    m_ui.tiltTopicCombo->clear();
    
    QStringList topicList;
    QStringList topics = Utils::getAllTopics("std_msgs/Float64");
    topics.append("");
    qSort(topics);
    for (QStringList::const_iterator it = topics.begin(); it != topics.end(); it++) {
      QString label(*it);
      label.replace(" ", "/");
      m_ui.panTopicCombo->addItem(label, QVariant(*it));
      m_ui.tiltTopicCombo->addItem(label, QVariant(*it));
      topicList.append(*it);
    }

    if(panTopic.isEmpty()) {
      for(int i = 0; i < topicList.count(); i++) {
        if(!topicList[i].contains("/delayed/")) {
          if(topicList[i].contains("pan")) {
            panTopic = topicList[i];
            break;
          }
        }
      }
    }
    if(tiltTopic.isEmpty()) {
      for(int i = 0; i < topicList.count(); i++) {
        if(!topicList[i].contains("/delayed/")) {
          if(topicList[i].contains("tilt")) {
            tiltTopic = topicList[i];
            break;
          }
        }
      }
    }
    // restore previous topics
    selectTopic(panTopic, m_ui.panTopicCombo);
    selectTopic(tiltTopic, m_ui.tiltTopicCombo);
  }

  void PanTilt::selectTopic(const QString& topic, QComboBox* combo)
  {
    int index = combo->findText(topic);
    if (index == -1)
    {
      // add topic name to list if not yet in
      QString label(topic);
      label.replace(" ", "/");
      combo->addItem(label, QVariant(topic));
      index = combo->findText(topic);
    }
    combo->setCurrentIndex(index);
  }

  void PanTilt::onPanTopicChanged(int index)
  {
    m_panPublisher.shutdown();
    std::string topicName = m_ui.panTopicCombo->currentText().toStdString();
    m_panPublisher = getNodeHandle().advertise<std_msgs::Float64>(topicName, 1000);
    m_ui.panTopicCombo->setToolTip(topicName.c_str());
  }

  void PanTilt::onTiltTopicChanged(int index) {
    m_tiltPublisher.shutdown();
    std::string topicName = m_ui.tiltTopicCombo->currentText().toStdString();
    m_tiltPublisher = getNodeHandle().advertise<std_msgs::Float64>(topicName, 1000);
    m_ui.tiltTopicCombo->setToolTip(topicName.c_str());
  }

  void PanTilt::onPanValueChanged(int value) {
    if(!m_panPublisher.getTopic().empty()) {
      std_msgs::Float64 msg;
      msg.data = 0.01 * value;
      m_panPublisher.publish(msg);
      m_ui.panValueBut->setText("Pan " + QString::number(msg.data));
    }
  }

  void PanTilt::onTiltValueChanged(int value) {
    if(!m_tiltPublisher.getTopic().empty()) {
      std_msgs::Float64 msg;
      msg.data = 0.01 * value;
      m_tiltPublisher.publish(msg);
      m_ui.tiltValueBut->setText("Tilt " + QString::number(msg.data));
    }
  }

  void PanTilt::onPanMinValueChanged(double value)  { 
    if(value < m_ui.panMaxSpin->value()) {
      m_ui.panSlider->setMinimum((int)(value*100)); 
    }
  }
  void PanTilt::onPanMaxValueChanged(double value) { 
    if(value > m_ui.panMinSpin->value()) {
      m_ui.panSlider->setMaximum((int)(value*100)); 
    }
  }

  void PanTilt::onTiltMinValueChanged(double value)  { 
    if(value < m_ui.tiltMaxSpin->value()) {
      m_ui.tiltSlider->setMinimum((int)(value*100)); 
    }
  }
  void PanTilt::onTiltMaxValueChanged(double value) { 
    if(value > m_ui.tiltMinSpin->value()) {
      m_ui.tiltSlider->setMaximum((int)(value*100)); 
    }
  }

  void PanTilt::onPanValuePressed() {
    m_panValueTime.restart();
  }
  
  void PanTilt::onTiltValuePressed() {
    m_tiltValueTime.restart();
  }
  
  void PanTilt::onPanValueReleased() {
    if(m_panValueTime.elapsed() < 2000) {
      m_ui.panSlider->setValue((int)(m_panValueDefault*100));
      onPanValueChanged(m_ui.panSlider->value());
    }
    else {
      m_panValueDefault = 0.01 * m_ui.panSlider->value();
      setDefaultTooltip(m_panValueDefault, m_ui.panValueBut);
    }
  }
  
  void PanTilt::onTiltValueReleased() {
    if(m_tiltValueTime.elapsed() < 2000) {
      m_ui.tiltSlider->setValue((int)(m_tiltValueDefault*100));
      onTiltValueChanged(m_ui.tiltSlider->value());
    }
    else {
      m_tiltValueDefault = 0.01 * m_ui.tiltSlider->value();
      setDefaultTooltip(m_tiltValueDefault, m_ui.tiltValueBut);
    }
  }
}

PLUGINLIB_EXPORT_CLASS(irg_rqt_tools::PanTilt, rqt_gui_cpp::Plugin)
