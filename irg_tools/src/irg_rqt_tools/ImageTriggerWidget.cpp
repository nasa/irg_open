#include "ImageTriggerWidget.h"

#include "std_msgs/msg/empty.hpp"

using namespace irg_rqt_tools;

ImageTriggerWidget::ImageTriggerWidget(rclcpp::Node::SharedPtr node, QString topic, QWidget* parent) : QWidget(parent) {
  setupUi(this);
  m_publisher = node->create_publisher<std_msgs::msg::Empty>(topic.toStdString(), 1000);

  QString triggerText = "/image_trigger";
  triggerBut->setText(topic.remove(triggerText));

  // hide options button because we don't have any options yet
  optionsBut->hide();
}

ImageTriggerWidget::~ImageTriggerWidget() {
}

void ImageTriggerWidget::on_triggerBut_clicked() {
  std_msgs::msg::Empty msg;
  m_publisher->publish(msg);
}

void ImageTriggerWidget::on_optionsBut_clicked() {
}
