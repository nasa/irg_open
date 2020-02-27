#include "ImageTrigger.h"

#include <cmath>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <QComboBox>
#include <QPushButton>
#include <QSpacerItem>
#include <QCursor>
#include <QToolTip>

#include "Utils.h"
#include "ImageTriggerWidget.h"

namespace irg_rqt_tools
{

  ImageTrigger::ImageTrigger()
    : rqt_gui_cpp::Plugin(),
      m_widget(0)
  {
    setObjectName("ImageTrigger");
  }

  void ImageTrigger::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    m_widget = new QWidget();
    m_ui.setupUi(m_widget);

    m_ui.refreshButton->setText("");
    m_ui.refreshButton->setIcon(QIcon::fromTheme("view-refresh"));
    
    int serialNum = context.serialNumber();
    if( serialNum > 1) {
      m_widget->setWindowTitle(m_widget->windowTitle() + " (" + QString::number(serialNum) + ")");
    }
    context.addWidget(m_widget);

    connect(m_ui.refreshButton, SIGNAL(clicked()), this, SLOT(refreshTopics()));
    
    refreshTopics();
  }

  void ImageTrigger::shutdownPlugin()
  {
  }

  void ImageTrigger::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
  {
  }

  void ImageTrigger::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
  {
  }
  
  void ImageTrigger::refreshTopics()
  {
    QLayoutItem* item;
    while((item = m_ui.triggerLayout->takeAt(0)) != NULL){
        delete item->widget();
        delete item;
    }
    QStringList triggerTopics = getTriggerTopics();
    for(int i = 0; i < triggerTopics.count(); i++) {
      ImageTriggerWidget* itw = new ImageTriggerWidget(getNodeHandle(), triggerTopics[i]);
      m_ui.triggerLayout->addWidget(itw);
    }
    QSpacerItem* spacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);
    m_ui.triggerLayout->addItem(spacer);
  }


  /**
   *
   */
  QStringList ImageTrigger::getTriggerTopics()
  {
    QStringList emptyTopics = Utils::getAllTopics("std_msgs/Empty");
    QStringList triggerTopics;
    for(int i = 0; i < emptyTopics.count(); i++) {
      if(emptyTopics[i].contains("image_trigger")) {
        triggerTopics.append(emptyTopics[i]);
      }
    }
    triggerTopics.sort();
    return triggerTopics;
  }
}

PLUGINLIB_EXPORT_CLASS(irg_rqt_tools::ImageTrigger, rqt_gui_cpp::Plugin)
