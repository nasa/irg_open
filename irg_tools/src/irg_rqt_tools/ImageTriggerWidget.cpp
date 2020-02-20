#include "ImageTriggerWidget.h"

#include <std_msgs/Empty.h>

using namespace irg_rqt_tools;

ImageTriggerWidget::ImageTriggerWidget(ros::NodeHandle& nodeHandle, QString topic, QWidget* parent)
  : 
  QWidget(parent)
{
  setupUi(this);
  QMetaObject::connectSlotsByName(this);
  m_publisher = nodeHandle.advertise<std_msgs::Empty>(topic.toStdString(), 1000);
  
  QString triggerText = "/image_trigger";
  triggerBut->setText(topic.remove(triggerText));
  
  // hide options button because we don't have any options yet
  optionsBut->hide();
}

ImageTriggerWidget::~ImageTriggerWidget() 
{
  m_publisher.shutdown();
}

void ImageTriggerWidget::on_triggerBut_clicked()
{
  std_msgs::Empty msg;
  m_publisher.publish(msg);
}

void ImageTriggerWidget::on_optionsBut_clicked()
{
  ROS_INFO("Options button clicked");
}
