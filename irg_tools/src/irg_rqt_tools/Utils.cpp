#include "Utils.h"

#include <ros/ros.h>

using namespace irg_rqt_tools;
  
/**
* get a list of *PUBLISHED* topics compatible with message type
* Note that this does not pick up subscribed topics
*/
QStringList Utils::getPublishedTopics(const QString& messageType) 
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QStringList topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++) {
    if(messageType.compare(it->datatype.c_str()) == 0) {
      QString topic = it->name.c_str();
      topics.append(topic);
    }
  }
  return topics;
}

/**
* This is more awkward, but gets us all topics - published and subscribed
*/
QStringList Utils::getAllTopics(const QString& messageType)
{
  QStringList retVal;
  XmlRpc::XmlRpcValue params("ros_topic_list");
  XmlRpc::XmlRpcValue results;
  XmlRpc::XmlRpcValue r;

  if(ros::master::execute("getTopicTypes", params, results, r, false) == true) {
    if(results.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      int32_t i = 2;
      if(results[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (int32_t j = 0; j < results[i].size(); ++j) {
          if(results[i][j].getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if(results[i][j].size() == 2) {
              if(results[i][j][0].getType() == XmlRpc::XmlRpcValue::TypeString
                  && results[i][j][1].getType() == XmlRpc::XmlRpcValue::TypeString) {
                std::string topic = static_cast<std::string>(results[i][j][0]);
                std::string type = static_cast<std::string>(results[i][j][1]);
                //std::cerr<<"Topic : "<<topic<<" -> "<<type<<std::endl;
                QString typeString(type.c_str());
                if(typeString == messageType) {
                  retVal.append(topic.c_str());
                }
              }
            }
          }
        }
      }
    }
  }
  return retVal;
}

