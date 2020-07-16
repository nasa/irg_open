// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <unistd.h>
#include <sys/stat.h>

#include <cstdlib>
#include <cstring>
#include <ctime>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "ephemeris.h"
#include "handle-options.h"

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
  const string format = "%Y-%m-%dT%H:%M:%S";
  char timestamp[64] = {0};
  strftime(timestamp, sizeof(timestamp), format.c_str(), gmtime(&epoch_time));

  // Now get the fractional component
  const long int NANOSECONDS_PER_MILLISECOND = 1000000;
  const long int milliseconds = ros_time.nsec / NANOSECONDS_PER_MILLISECOND;

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

geometry_msgs::TransformStamped
make_time_stamped_transform(const std::string &reference_body,
			    const std::string &target_body,
			    const ros::Time &ros_time,
			    float64_ow spice_transform[16])
{
  const double KM_TO_M = 1000.0; // Convert from kilometers to meters
  string reference_frame_name = reference_body, target_frame_name = target_body;
  
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
         ros::Publisher& sun_visibility_pub,
		     const string& reference_body,
		     const float64_ow lat, const float64_ow lon,
		     const float64_ow elev,
		     const vector<string> &target_bodies,
		     const ros::Time &ros_time,
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

    geometry_msgs::TransformStamped time_stamped_transform_msg =
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

    std_msgs::Float64 msg;
    msg.data = fraction_visible;
    sun_visibility_pub.publish(msg);
  }
}

static int
path_exists(const char *path)
{
  struct stat path_status;

  return (stat(path, &path_status) == 0);
}

// Deprecated
bool
have_run_parameters_file(int argc, char *argv[], ros::NodeHandle &nodeHandle,
			 string &run_parameters_filename)
{
  run_parameters_filename = "run_parameters.yaml";
    
  // Check for run parameter file arg passed via rosrun or roslaunch:
  if (!ros::param::get("~run_parameters_file", run_parameters_filename))
  {
    // Check for non-ROS command line args
    Options options;
    options.Add('f', "run parameters file name.\n", "string", 1);
    int firstArgIndex = HandleOptions(argc, argv, options, 0, "");
    if (options.GetOptionInfo('f')->IsSet())
      run_parameters_filename = options.GetOptionInfo('f')->GetValuesString();
  }
  if (!path_exists(run_parameters_filename.c_str()))
  {
    run_parameters_filename = "";
    return false;
  }

  return true;
}

bool
have_non_ros_paramater_file_option(int argc, char *argv[],
				   string &run_parameters_filename)
{
  run_parameters_filename = "";
  
  // Check for non-ROS command line args
  Options options;
  options.Add('f', "run parameters file name.\n", "string", 1);
  int firstArgIndex = HandleOptions(argc, argv, options, 0, "");
  if (!options.GetOptionInfo('f')->IsSet())
    return false;
  
  run_parameters_filename = options.GetOptionInfo('f')->GetValuesString();
  if (!path_exists(run_parameters_filename.c_str()))
  {
    run_parameters_filename = "";
    return false;
  }

  return true;
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
read_run_parameters(const string &run_parameters_filename, string &reference_body,
		    vector<string> &target_bodies,
		    float64_ow &mission_lat, float64_ow &mission_lon,
		    float64_ow &mission_elev, bool &z_down_surface_frame,
		    string &leapSecondKernelPath, string &constantsKernelPath, 
		    vector<string> &ephemerisPaths)
{
  
  YAML::Node parameters = YAML::LoadFile(run_parameters_filename);
  if (parameters["reference_body"])
    reference_body = parameters["reference_body"].as<string>();
  else
    return false;
  if (parameters["target_bodies"])
  {
    YAML::Node target_bodies_node = parameters["target_bodies"];
    for (std::size_t i = 0; i < target_bodies_node.size(); i++) 
      target_bodies.push_back(target_bodies_node[i].as<string>());
  }  
  else
    return false;
  if (parameters["mission_lat"])
    mission_lat = parameters["mission_lat"].as<double>();
  else
    return false;
  if (parameters["mission_lon"])
    mission_lon = parameters["mission_lon"].as<double>();
  else
    return false;
  mission_elev = 0.0;
  if (parameters["mission_elev"])
    mission_elev = parameters["mission_elev"].as<double>();
  else
    cerr << "WARNING [read_run_parameters()]: "
	 << "no mission elevation specified, assuming elevation = 0" << endl;
  z_down_surface_frame = true;
  if (parameters["z_down_surface_frame"])
    mission_elev = parameters["z_down_surface_frame"].as<bool>();
  else
    cerr << "WARNING [read_run_parameters()]: "
	 << "surface frame Z axis direction not specified, assuming Z down."
	 << endl;
  if (parameters["leap_second_kernel"])
    leapSecondKernelPath = parameters["leap_second_kernel"].as<string>();
  else
    return false;
  if (parameters["constants_kernel"])
    constantsKernelPath = parameters["constants_kernel"].as<string>();
  else
    return false;
  if (parameters["ephemerides"])
  {
    YAML::Node ephemerides_node = parameters["ephemerides"];
    for (std::size_t i = 0; i < ephemerides_node.size(); i++) 
      ephemerisPaths.push_back(ephemerides_node[i].as<string>());
  }
  else
    return false;

  return true;
}

bool
read_ros_run_parameters(string &reference_body,
			vector<string> &target_bodies,
			float64_ow &mission_lat, float64_ow &mission_lon,
			float64_ow &mission_elev, bool &z_down_surface_frame,
			string &leapSecondKernelPath, string &constantsKernelPath, 
			vector<string> &ephemerisPaths)
{
  if (!ros::param::get("~reference_body", reference_body))
    return false;
  
  if (!ros::param::get("~target_bodies", target_bodies))
    return false;

  if (!ros::param::get("~mission_lat", mission_lat))
    return false;

  if (!ros::param::get("~mission_lon", mission_lon))
    return false;

  mission_elev = 0.0;
  if (!ros::param::get("~mission_elev", mission_elev))
    ROS_INFO_STREAM("WARNING [read_ros_run_parameters()]: "
        "no mission elevation specified, assuming elevation = 0");
  z_down_surface_frame = true;
  if (!ros::param::get("~z_down_surface_frame", z_down_surface_frame))
    ROS_INFO_STREAM("WARNING [read_ros_run_parameters()]: "
        "surface frame Z axis direction not specified, "
        "assuming Z down.");

  if (!ros::param::get("~leap_second_kernel", leapSecondKernelPath))
    return false;

  if (!ros::param::get("~constants_kernel", constantsKernelPath))
    return false;

  if (!ros::param::get("~ephemerides", ephemerisPaths))
    return false;

  return true;
}

int
main(int argc, char *argv[])
{
  // Check for required environment variable and gracefully exit if
  // unset
  check_ros_environment();
  
  // ROS initialization
  ros::init(argc, argv, "ephemeris_publisher_node");
  ros::NodeHandle nodeHandle;
  tf2_ros::TransformBroadcaster broadcaster;
  // Set the transform update rate
  ROS_INFO_STREAM("Starting ephemeris publisher node!");
  string publishPeriodString;
  int publishPeriod;
  if (ros::param::get("~ephemeris_publisher_period", publishPeriodString))
    publishPeriod = atoi(publishPeriodString.c_str());
  else
    publishPeriod = 5; // The default period
  ROS_INFO_STREAM("Setting publish period to " << publishPeriod);

  ros::Publisher sun_visibility_pub = nodeHandle.advertise<std_msgs::Float64>("sun_visibility", 1);

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
  if (have_non_ros_paramater_file_option(argc, argv, run_parameters_filename))
  {
    if (!read_run_parameters(run_parameters_filename,
           reference_body, target_bodies,
           mission_lat, mission_lon, mission_elev,
           z_down_surface_frame,
           leapSecondKernelPath, constantsKernelPath,
           ephemerisPaths))
    {
      cerr << "FATAL ERROR [main()]: "
     << "unable to read run time parameters file. Exiting." << endl;
      exit(-1);
    }
  }
  else
  {
    if (!read_ros_run_parameters(reference_body, target_bodies,
         mission_lat, mission_lon, mission_elev,
         z_down_surface_frame,
         leapSecondKernelPath, constantsKernelPath,
         ephemerisPaths))
    {
      cerr << "FATAL ERROR [main()]: "
     << "unable to read all ROS run time parameters. Exiting." << endl;
      exit(-1);
    }
  }

  Ephemeris ephemeris(leapSecondKernelPath, constantsKernelPath, ephemerisPaths, z_down_surface_frame);
  
  // Broadcasting loop
  bool first_error = true;
  ros::Rate publishRate(publishPeriod);
  ROS_INFO_STREAM("Entering broadcasting loop...");
  while (ros::ok())
  {
    // Get the current time and make sure it is reasonable.
    ros::Time current_time = ros::Time::now();
    if (current_time.sec < 400000000) // An arbitrary time very far from 0!
    {
      if (!first_error) // This always happens once in sim so suppress
      // the first error message.
        ROS_ERROR_STREAM("Got low value for ros::Time::now() in the solar frame publisher: "
                         << current_time);
      first_error = false;
      sleep(3); // Wait to see if a valid time is published
      continue;
    }

    broadcast_transforms(broadcaster, sun_visibility_pub, reference_body,
                         mission_lat, mission_lon, mission_elev,
                         target_bodies, current_time, ephemeris);
    
    publishRate.sleep();
  }
  
  ROS_INFO_STREAM("Program stopped.");
}
