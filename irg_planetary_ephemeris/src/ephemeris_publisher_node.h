// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef EPHEMERIS_PUBLISHER_NODE_H_
#define EPHEMERIS_PUBLISHER_NODE_H_

#include "ephemeris.h"


namespace ow
{
  class EphemerisPublisherNode
  {
  public:
    EphemerisPublisherNode()
    {
      ros::NodeHandle nodeHandle;
      // Set the transform update rate
      std::string publishPeriodString;
      int publishPeriod;
      if (nodeHandle.getParam("ephemeris_publisher_period", publishPeriodString))
	publishPeriod = atoi(publishPeriodString.c_str());
      else
	publishPeriod = 5; // The default period
      ROS_INFO_STREAM("Setting publish period to " << publishPeriod);
      ros::Rate publishRate(publishPeriod);
    }
  private:
    Ephemeris m_ephemeris;
    tf2_ros::TransformBroadcaster m_broadcaster;
    tf2_ros::StaticTransformBroadcaster m_static_broadcaster;
  };
}

#endif // EPHEMERIS_PUBLISHER_NODE_H_
