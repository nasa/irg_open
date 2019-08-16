#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <ctime>

#include <yaml.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "ephemeris.h"

using namespace std;
using namespace ow;


static void
check_ros_environment()
{
  const char *ros_master_uri = getenv("ROS_MASTER_URI");

  if (ros_master_uri == NULL)
  {
    cerr << "\nFATAL ERROR [main()]: "
	 << "ROS_MASTER_URI is not defined in the environment.\n"
	 << "To set up your local machine as a ROS master:\n\n"
	 << "setenv ROS_MASTER_URI http://localhost:11311\n" << endl;
    exit(-1);
  }
}

inline std::string ros_time_to_string(ros::Time ros_time)
{
  // Convert epoch time to time_t format
  time_t epoch_time = ros_time.sec;

  // Convert to a string
  const std::string format = "%Y-%m-%dT%H:%M:%S";
  char timestamp[64] = {0};
  strftime(timestamp, sizeof(timestamp), format.c_str(), gmtime(&epoch_time));

  // Now get the fractional component
  const long int NANOSECONDS_PER_MILLISECOND = 1000000;
  const long int milliseconds = ros_time.nsec / NANOSECONDS_PER_MILLISECOND;

  std::stringstream time_string;
  time_string << timestamp << "." << milliseconds;

  return time_string.str();
}

geometry_msgs::TransformStamped
make_time_stamped_transform(const std::string &reference_body,
			    const std::string &target_body,
			    const ros::Time &ros_time,
			    float64_ow spice_transform[16])
{
  const std::string RP_MOON_NAME  = "moon_frame";
  const std::string RP_EARTH_NAME = "earth_frame";
  const std::string RP_SUN_NAME   = "sun_frame";
  const double KM_TO_M = 1000.0; // Convert from kilometers to meters

  string reference_frame_name, target_frame_name;
  
  geometry_msgs::Transform transform_msg;
  geometry_msgs::TransformStamped time_stamped_transform_msg;

  tf2::Matrix3x3 rotation_matrix(spice_transform[0], spice_transform[1], spice_transform[2],
				 spice_transform[4], spice_transform[5], spice_transform[6], 
				 spice_transform[8], spice_transform[9], spice_transform[10]);
  tf2::Transform temp_transform;
  temp_transform.setBasis(rotation_matrix);
  tf2::Quaternion q = temp_transform.getRotation();
  q.normalize();
  
  // Pack transform into a transform message
  transform_msg.translation.x = spice_transform[3] * KM_TO_M;
  transform_msg.translation.y = spice_transform[7] * KM_TO_M;
  transform_msg.translation.z = spice_transform[11] * KM_TO_M;
  transform_msg.rotation.x = q.x();
  transform_msg.rotation.y = q.y();
  transform_msg.rotation.z = q.z();
  transform_msg.rotation.w = q.getW();
  
  // Pack into the output timestamped transform message
  time_stamped_transform_msg.transform = transform_msg;
  time_stamped_transform_msg.header.stamp = ros_time;
  // The parent in the node tree:
  time_stamped_transform_msg.header.frame_id = reference_frame_name;
  // The child in the node tree:
  time_stamped_transform_msg.child_frame_id  = target_frame_name;

  return time_stamped_transform_msg;
}

void
broadcast_transforms(tf2_ros::TransformBroadcaster broadcaster,
		     const string& reference_body,
		     const float64_ow lat, const float64_ow lon,
		     const vector<string> &target_bodies,
		     const ros::Time &ros_time,
		     Ephemeris &ephemeris)
{
  string time_string = ros_time_to_string(ros_time);

  for (int i = 0; i < target_bodies.size(); ++i)
  {
    float64_ow spice_transform[16];

    ephemeris.SurfaceToTargetBodyTransform(reference_body, lat, lon, 
					   target_bodies[i], time_string,
					   spice_transform);

    geometry_msgs::TransformStamped time_stamped_transform_msg =
      make_time_stamped_transform(reference_body, target_bodies[i], 
				  ros_time, spice_transform);

    // Publish the time stamped transforms for this target body
    broadcaster.sendTransform(time_stamped_transform_msg);
  }
}

int
main(int argc, char *argv[])
{
  // Check for required environment variable and gracefully exit if
  // unset
  check_ros_environment();
  
  // Set up ROS
  ros::init(argc, argv, "ephemeris_publisher_node");
  ros::NodeHandle nodeHandle;

  // Set the transform update rate
  ROS_INFO_STREAM("Starting ephemeris publisher node!");
  std::string publishPeriodString;
  int publishPeriod;
  if (nodeHandle.getParam("ephemeris_publisher_period", publishPeriodString))
    publishPeriod = atoi(publishPeriodString.c_str());
  else
    publishPeriod = 5; // The default period
  ROS_INFO_STREAM("Setting publish period to " << publishPeriod);

  tf2_ros::TransformBroadcaster broadcaster;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  bool first_error = true;
  ros::Rate publishRate(publishPeriod);
  ROS_INFO_STREAM("Entering broadcasting loop...");
  while (ros::ok())
  {
    // Get the current time and make sure it is reasonable.    
    ros::Time current_time = ros::Time::now();
    if (current_time.sec < 400000000) // An arbitrary time very far from 0! 
    {
      if (!first_error) // This always happens once in sim so suppress the first error message.
        ROS_ERROR_STREAM("Got low value for ros::Time::now() in the solar frame publisher: "
                         << current_time);
      first_error = false;
      sleep(3); // Wait to see if a valid time is published
      continue;
    }
    
    
    publishRate.sleep();
  }
  
  ROS_INFO_STREAM("Program stopped.");
}
