// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef EPHEMERIS_SERVICE_NODE_H_
#define EPHEMERIS_SERVICE_NODE_H_

#include "ephemeris.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace ow
{
  class EphemerisNode
  {
  public:
    EphemerisNode(ros::NodeHandle nodeHandle)
    {
      ros::Publisher pub = nodeHandle.advertise<std_msgs::String>("vector_to_planetary_body", 5);
      ros::Subscriber sub = nodeHandle.subscribe("query_vector_to_planetary_body", 1, &EphemerisNode::callback, this);
    }
    void callback(const std_msgs::String::ConstPtr& message);
    ros::Publisher pub;
    ros::Subscriber sub;
  private:
    Ephemeris m_ephemeris;
  };
}

#endif // EPHEMERIS_SERVICE_NODE_H_
