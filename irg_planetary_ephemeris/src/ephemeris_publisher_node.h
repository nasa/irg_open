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
      //TODO: If this is used somewhere, probably want to pass in this node handle!
      m_node_handle = rclcpp::Node::make_shared("ephemeris_publisher");
      // Set the transform update rate
      std::string publishPeriodString;
      int publishPeriod;
      if (m_node_handle->getParam("ephemeris_publisher_period", publishPeriodString))
        publishPeriod = atoi(publishPeriodString.c_str());
      else
        publishPeriod = 5; // The default period
      RCLCPP_INFO(m_node_handle->get_logger(),
                  "Setting publish period to " << publishPeriod);
      rclcpp::Rate publishRate(publishPeriod);
    }
  private:
    rclcpp::Node::SharedPtr m_node_handle;
    Ephemeris m_ephemeris;
    tf2_ros::TransformBroadcaster m_broadcaster;
    tf2_ros::StaticTransformBroadcaster m_static_broadcaster;
  };
}

#endif // EPHEMERIS_PUBLISHER_NODE_H_
