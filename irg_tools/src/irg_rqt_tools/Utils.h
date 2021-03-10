#ifndef irg_rqt_tools_Utils_h
#define irg_rqt_tools_Utils_h

#include <string>
#include <regex>

#include <QString>
#include <QStringList>
#include <rclcpp/rclcpp.hpp>

namespace irg_rqt_tools {

  class Utils {
  public:
    static QStringList getAllTopics(rclcpp::Node::SharedPtr node, const QString& messageType);
    static QStringList findMatchingNodeNames(rclcpp::Node::SharedPtr node, const std::string& regexStr);
  };

}

#endif // irg_rqt_tools_Utils_h
