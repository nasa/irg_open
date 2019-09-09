
#include "ephemeris_service_node.h"

using namespace ow;
using namespace std;

void EphemerisNode::callback(const std_msgs::msg::String::SharedPtr message)
{
  string referenceBodyName;
  string targetBodyName;
  string time;
  float64_ow latitude, longitude;
  float64_ow target_vector[3];
  
  m_ephemeris.VectorToTarget(referenceBodyName, latitude, longitude,
			     targetBodyName, time, target_vector);

  std_msgs::msg::String str;
  str.data = "hello world";
  pub.publish(str);
}

static void
check_ros_environment()
{
  const char *ros_master_uri = getenv("ROS_MASTER_URI");

  if (ros_master_uri == NULL)
  {
    cerr << "\nFATAL ERROR [main()]: ROS_MASTER_URI is not defined in the environment.\n"
	 << "To set up your local machine as a ROS master:\n\n"
	 << "setenv ROS_MASTER_URI http://localhost:11311\n" << endl;
    exit(-1);
  }
}

int main(int argc, char *argv[])
{
  check_ros_environment();
  
  // Set up ROS
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("ephemeris_node");

  EphemerisNode ephemerisNode(nodeHandle);
  
  RCLCPP_INFO(nodeHandle->get_logger(), "Starting ephemeris node!");

  RCLCPP_INFO(nodeHandle->get_logger(), "Ephemeris node listening...");

  // Wait for target vector queries
  rclcpp::spin();
  
  RCLCPP_INFO(nodeHandle->get_logger(), "Ephemeris node stopped.");
}
