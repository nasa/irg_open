// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <unistd.h>
#include <sys/stat.h>

#include <cstdlib>
#include <cstring>
#include <ctime>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "ephemeris.h"

using namespace std;
using namespace ow;


/// Convert a ros::Time into a builtin_interfaces::msg::Time
inline builtin_interfaces::msg::Time ros_time_to_builtin_time(rclcpp::Time ros_time)
{
  const long unsigned int nanosec_per_sec = 1000LL * 1000LL * 1000LL;

  builtin_interfaces::msg::Time output;
  unsigned long int nsec   = ros_time.nanoseconds();
  unsigned long int sec    = nsec / nanosec_per_sec;
  unsigned long int sec_ns = sec * nanosec_per_sec;
  output.nanosec = nsec - sec_ns;
  output.sec = sec;

  return output;
}


inline std::string ros_time_to_string(builtin_interfaces::msg::Time ros_time)
{
  // Convert epoch time to time_t format
  time_t epoch_time = ros_time.sec;

  // Convert to a string
  const string format = "%Y-%m-%dT%H:%M:%S";
  char timestamp[64] = {0};
  strftime(timestamp, sizeof(timestamp), format.c_str(), gmtime(&epoch_time));

  // Now get the fractional component
  const long int NANOSECONDS_PER_MILLISECOND = 1000000;
  const long int milliseconds = ros_time.nanosec / NANOSECONDS_PER_MILLISECOND;

  std::stringstream time_string;
  time_string << timestamp << "." << milliseconds;

  return time_string.str();
}

// Deprecated
string tolower_string(string the_string)
{
  transform(the_string.begin(), the_string.end(), the_string.begin(), 
	    [](unsigned char ch){ return tolower(ch); } );
  return the_string;
}

geometry_msgs::msg::TransformStamped
make_time_stamped_transform(const std::string &reference_body,
			    const std::string &target_body,
			    const builtin_interfaces::msg::Time &ros_time,
			    float64_ow spice_transform[16])
{
  const double KM_TO_M = 1000.0; // Convert from kilometers to meters
  string reference_frame_name = reference_body, target_frame_name = target_body;
  
  geometry_msgs::msg::Transform transform_msg;
  geometry_msgs::msg::TransformStamped time_stamped_transform_msg;

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
broadcast_transforms(tf2_ros::TransformBroadcaster& broadcaster,
         rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sun_visibility_pub,
		     const string& reference_body,
		     const float64_ow lat, const float64_ow lon,
		     const float64_ow elev,
		     const vector<string> &target_bodies,
		     const builtin_interfaces::msg::Time &ros_time,
		     Ephemeris &ephemeris)
{
  string time_string = ros_time_to_string(ros_time);

  int sun_index = -1;

  for (int i = 0; i < target_bodies.size(); ++i)
  {
    float64_ow spice_transform[16];

    ephemeris.SurfaceToTargetBodyTransform(reference_body, lat, lon, elev,
					   target_bodies[i], time_string,
					   spice_transform);

    geometry_msgs::msg::TransformStamped time_stamped_transform_msg =
      make_time_stamped_transform(reference_body, target_bodies[i], 
				  ros_time, spice_transform);

    // Publish the time stamped transforms for this target body
    broadcaster.sendTransform(time_stamped_transform_msg);

    // Is this the sun?
    std::string lowercase_name = target_bodies[i];
    std::transform(lowercase_name.begin(), lowercase_name.end(), lowercase_name.begin(),
        [](unsigned char c){ return std::tolower(c); });
    if (lowercase_name == "sun")
    {
      sun_index = i;
    }
  }

  // Compute amount of sun occultation by another body.
  // This algorithm finds the minimum value output by simple body-to-body
  // occultation events. It does not handle the extremely unlikely case of
  // occultation by multiple bodies simultaneously.
  double fraction_visible = 1.0;
  if (sun_index >= 0 && target_bodies.size() > 1)
  {
    for (int i = 0; i < target_bodies.size(); ++i)
    {
      if (i == sun_index)
      {
        continue;
      }

      const double current_fv = ephemeris.FractionVisible(target_bodies[sun_index],
                                                          target_bodies[i]);
      if(current_fv < fraction_visible){
        fraction_visible = current_fv;
      }
    }

    std_msgs::msg::Float64 msg;
    msg.data = fraction_visible;
    sun_visibility_pub->publish(msg);
  }
}

void
set_default_run_parameters(string &reference_body,
			   vector<string> &target_bodies,
			   float64_ow &mission_lat, float64_ow &mission_lon,
			   float64_ow &mission_elev,
			   bool &z_down_surface_frame,
			   string &leapSecondKernelPath,
			   string &constantsKernelPath,
			   vector<string> &ephemerisPaths)
{
  reference_body = "MOON";
  target_bodies.push_back("EARTH");
  target_bodies.push_back("SUN");
  mission_lat = 0.0;
  mission_lon = 0.0;
  mission_elev = 0.0;
  z_down_surface_frame = true;
  leapSecondKernelPath = "./latest_leapseconds.tls";
  constantsKernelPath =  "./pck00010.tpc";
  ephemerisPaths = { "./de430.bsp" };
}

bool
read_ros_run_parameters(rclcpp::Node::SharedPtr nodeHandle,
                        string &reference_body, vector<string> &target_bodies,
                        float64_ow &mission_lat, float64_ow &mission_lon,
                        float64_ow &mission_elev, bool &z_down_surface_frame,
                        string &leapSecondKernelPath, string &constantsKernelPath,
                        vector<string> &ephemerisPaths)
{
  if (!nodeHandle->get_parameter("reference_body", reference_body))
    return false;
  
  if (!nodeHandle->get_parameter("target_bodies", target_bodies))
    return false;
  
  if (!nodeHandle->get_parameter("mission_lat", mission_lat))
    return false;
  
  if (!nodeHandle->get_parameter("mission_lon", mission_lon))
    return false;

  mission_elev = 0.0;
  if (!nodeHandle->get_parameter("mission_elev", mission_elev))
    RCLCPP_INFO(nodeHandle->get_logger(),
                "WARNING [read_ros_run_parameters()]: "
                "no mission elevation specified, assuming elevation = 0");
  z_down_surface_frame = true;
  if (!nodeHandle->get_parameter("z_down_surface_frame", z_down_surface_frame))
    RCLCPP_INFO(nodeHandle->get_logger(),
                "WARNING [read_ros_run_parameters()]: "
		          "surface frame Z axis direction not specified, "
		          "assuming Z down.");

  if (!nodeHandle->get_parameter("leap_second_kernel", leapSecondKernelPath))
    return false;

  if (!nodeHandle->get_parameter("constants_kernel", constantsKernelPath))
    return false;

  if (!nodeHandle->get_parameter("ephemerides", ephemerisPaths))
    return false;

  return true;
}

int
main(int argc, char *argv[])
{ 
  // ROS initialization
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  // params in YAML file are implicitly declared (like ROS1 behavior)
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("ephemeris_publisher_node", options);
  tf2_ros::TransformBroadcaster broadcaster(nodeHandle);
  // Set the transform update rate
  RCLCPP_INFO(nodeHandle->get_logger(),
              "Starting ephemeris publisher node!");
  string publishPeriodString;
  int publishPeriod;
  if (nodeHandle->get_parameter("ephemeris_publisher_period", publishPeriodString))
    publishPeriod = atoi(publishPeriodString.c_str());
  else
    publishPeriod = 5; // The default period
  RCLCPP_INFO(nodeHandle->get_logger(),
              "Setting publish period to %d", publishPeriod);

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sun_visibility_pub =
      nodeHandle->create_publisher<std_msgs::msg::Float64>("sun_visibility", 1);

  string run_parameters_filename;
  string reference_body;
  vector<string> target_bodies;
  float64_ow mission_lat, mission_lon, mission_elev;
  bool z_down_surface_frame;
  string leapSecondKernelPath, constantsKernelPath;
  vector<string> ephemerisPaths;

  set_default_run_parameters(reference_body, target_bodies,
			     mission_lat, mission_lon, mission_elev,
			     z_down_surface_frame,
			     leapSecondKernelPath, constantsKernelPath, 
			     ephemerisPaths);
  if (!read_ros_run_parameters(nodeHandle, reference_body, target_bodies,
       mission_lat, mission_lon, mission_elev,
       z_down_surface_frame,
       leapSecondKernelPath, constantsKernelPath,
       ephemerisPaths))
  {
    cerr << "FATAL ERROR [main()]: "
    << "unable to read all ROS run time parameters. Exiting." << endl;
    exit(-1);
  }

  Ephemeris ephemeris(leapSecondKernelPath, constantsKernelPath, ephemerisPaths, z_down_surface_frame);
  
  // Broadcasting loop
  rclcpp::Rate publishRate(publishPeriod);
  RCLCPP_INFO(nodeHandle->get_logger(),
              "Entering broadcasting loop...");
  while (rclcpp::ok())
  {
    // Get the current time and make sure it is reasonable.    
    rclcpp::Time current_time = nodeHandle->get_clock()->now();
    if (current_time.nanoseconds() < 400000000000) // An arbitrary time far from 0!
    {
      RCLCPP_ERROR_SKIPFIRST(nodeHandle->get_logger(),
                             "Got low value for ros::Time::now() in the solar frame publisher: %.3f",
                             current_time.seconds());
      sleep(3); // Wait to see if a valid time is published
    }
    else
    {
      builtin_interfaces::msg::Time builtin_time = ros_time_to_builtin_time(current_time);
      broadcast_transforms(broadcaster, sun_visibility_pub, reference_body,
                           mission_lat, mission_lon, mission_elev,
                           target_bodies, builtin_time, ephemeris);
    }

    // Make params visible to `ros2 param list` and update the clock
    rclcpp::spin_some(nodeHandle);

    publishRate.sleep();
  }

  RCLCPP_INFO(nodeHandle->get_logger(), "Program stopped.");
}
