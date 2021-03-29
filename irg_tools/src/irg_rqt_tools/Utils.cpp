#include "Utils.h"

#include <vector>

using namespace irg_rqt_tools;
  
QStringList
Utils::getAllTopics(rclcpp::Node::SharedPtr node, const QString& messageType)
{
  QStringList topics;
  auto name_type_map = node->get_topic_names_and_types();

  for (std::map<std::string, std::vector<std::string>>::iterator it = name_type_map.begin(); it != name_type_map.end(); it++) {
    for (auto s : it->second) {
      if (s == messageType.toStdString()) {
        topics.append(QString::fromStdString(it->first));
        break;
      }
    }
  }

  return topics;
}

QStringList
Utils::findMatchingNodeNames(rclcpp::Node::SharedPtr node, const std::string& regexStr)
{
  QStringList filtered;
  auto node_names = node->get_node_names();

  for (auto &name : node_names) {
    if (regex_match(name, std::regex(regexStr))) {
      filtered.append(QString::fromStdString(name));
    }
  }

  return filtered;
}
