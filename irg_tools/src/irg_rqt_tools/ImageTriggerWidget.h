#ifndef irg_rqt_tools_ImageTriggerWidget_h
#define irg_rqt_tools_ImageTriggerWidget_h

#include "rclcpp/rclcpp.hpp"
#include "ui_ImageTriggerWidget.h"
#include "std_msgs/msg/empty.hpp"

namespace irg_rqt_tools
{
class ImageTriggerWidget : public QWidget, protected Ui::ImageTriggerButton
{
  Q_OBJECT
public:
  ImageTriggerWidget(rclcpp::Node::SharedPtr node, QString topic, QWidget* parent = 0);
  ~ImageTriggerWidget();

protected slots:
  void on_triggerBut_clicked();
  void on_optionsBut_clicked();

protected:
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_publisher;
};

}

#endif // irg_rqt_tools_ImageTriggerWidget_h
