#ifndef irg_rqt_tools_Utils_h
#define irg_rqt_tools_Utils_h

#include <QString>
#include <QStringList>

#include "rclcpp/rclcpp.hpp"

namespace irg_rqt_tools {

  class Utils {
  public:
    static QStringList getAllTopics(rclcpp::Node::SharedPtr node, const QString& messageType);
  };

}

#endif // irg_rqt_tools_Utils_h
