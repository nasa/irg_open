#ifndef irg_rqt_tools_ImageTriggerWidget_h
#define irg_rqt_tools_ImageTriggerWidget_h

#include <ros/ros.h>
#include "ui_ImageTriggerWidget.h"

namespace irg_rqt_tools {
  
  class ImageTriggerWidget : public QWidget, protected Ui::ImageTriggerButton
  {
    Q_OBJECT
  public:
    ImageTriggerWidget(ros::NodeHandle& nodeHandle, QString topic, QWidget* parent = 0);
    ~ImageTriggerWidget();

  protected slots:
    void on_triggerBut_clicked();
    void on_optionsBut_clicked();
    
  protected:
    ros::Publisher    m_publisher;
  };

  
}

#endif // irg_rqt_tools_ImageTriggerWidget_h
