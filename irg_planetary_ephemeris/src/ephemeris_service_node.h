// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef EPHEMERIS_SERVICE_NODE_H_
#define EPHEMERIS_SERVICE_NODE_H_

#include "ephemeris.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ow
{
  class EphemerisNode
  {
  public:
    EphemerisNode(rclcpp::Node::SharedPtr nodeHandle)
    {
      pub = nodeHandle->create_publisher<std_msgs::msg::String>("/rgsw/vo/icp", 10);
      sub = nodeHandle->subscribe("query_vector_to_planetary_body",
                                  std::bind(&EphemerisNode::callback, this, std::placeholders::_1));
    }
    void callback(const std_msgs::String::SharedPtr message);

    ros::Publisher pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr sub;
  private:
    Ephemeris m_ephemeris;
  };
}

#endif // EPHEMERIS_SERVICE_NODE_H_
