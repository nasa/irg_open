// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "CameraCompositorListener.h"
#include <gazebo/common/Assert.hh>


using namespace irg;


CameraCompositorListener::CameraCompositorListener(sdf::ElementPtr sdf)
{
  m_node = gazebo_ros::Node::Get(sdf);

  // Add callback for setting of rosparams
  auto rosparam_callback = [this](const std::vector<rclcpp::Parameter>& rosparams) {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (const auto & rosparam : rosparams) {
      const std::string& rosparam_name = rosparam.get_name();
      if (m_param_map.find(rosparam_name) != m_param_map.end()) {
        m_param_map[rosparam_name].m_value =
            static_cast<Ogre::Real>(rosparam.as_double());
      }
    }
    return result;
  };
  m_rosparam_callback = m_node->add_on_set_parameters_callback(rosparam_callback);

  // Get UID for topic from the sdf element
  if (sdf->HasElement("topic_uid")) {
    m_topic_uid = sdf->Get<std::string>("topic_uid");

    // Remove all single- and double-quotes, just in case the user used any.
    m_topic_uid.erase(remove( m_topic_uid.begin(), m_topic_uid.end(), '\'' ), m_topic_uid.end());
    m_topic_uid.erase(remove( m_topic_uid.begin(), m_topic_uid.end(), '\"' ), m_topic_uid.end());
  }

  // Declare shader param names and default values
  initParam(sdf, "lens_transmission", 1.0);
  initParam(sdf, "exposure", 1.0);
  initParam(sdf, "energy_conversion", 1.0);
  initParam(sdf, "read_noise", 0.64);
  initParam(sdf, "shot_noise", 0.09);
  initParam(sdf, "gain", 1.0);
  initParam(sdf, "gamma", 1.0);
  initParam(sdf, "adc_bits", 12.0);
}

void CameraCompositorListener::initParam(sdf::ElementPtr sdf, const std::string& name, double initial_value)
{
  if (sdf->HasElement(name)) {
    initial_value = sdf->Get<Ogre::Real>(name);
  }

  m_param_map[name].m_value = initial_value;

  // create topic name
  std::string topic("/gazebo/plugins/camera_sim/");
  if (!m_topic_uid.empty())
  {
    topic += m_topic_uid;
    topic += "/";
  }
  topic += name;

  // This is an official workaround, poorly described in NodeHandle reference
  // pages and many answers.ros.org discussions, for passing extra variables to
  // subscriber callbacks. This allows us to have one callback instead of one
  // for each param (shader uniform) we want to set.
  // You had to use boost::function in ROS1, but you must use std::function in ROS2.
  // You must pass your topic type as reference (don't forget the ampersand).
  // Commented out is an example of how you can also use a lambda
  // to achieve the same result. I'm using std::bind simply to be consistent
  // with std::function.
  std::function<void (const std_msgs::msg::Float64::SharedPtr msg)> func =
    std::bind(&CameraCompositorListener::onParamUpdate, this, std::placeholders::_1, name);
    //[this, name](const std_msgs::Float64::ConstPtr& msg){ m_param_map[name].m_value = msg->data; };

  // Subscribe using our fancy bound function pointer.
  m_param_map[name].m_subscriber = m_node->create_subscription<std_msgs::msg::Float64>(topic, 1, func);

  // Declare as a rosparam. (This will invoke m_rosparam_callback)
  m_node->declare_parameter(name, initial_value);
}

void CameraCompositorListener::onParamUpdate(const std_msgs::msg::Float64::SharedPtr msg, const std::string& name)
{
  m_node->set_parameter(rclcpp::Parameter(name, msg->data));
}

void CameraCompositorListener::notifyMaterialRender(const unsigned int pass_id, Ogre::MaterialPtr& mat)
{
  GZ_ASSERT(!mat.isNull(), "Null Ogre3D material");
  Ogre::Technique* technique = mat->getTechnique(0);
  GZ_ASSERT(technique != nullptr, "Null Ogre3D technique");
  Ogre::Pass* pass = technique->getPass(pass_id);
  GZ_ASSERT(pass != nullptr, "Null Ogre3D pass");
  Ogre::GpuProgramParametersSharedPtr params = pass->getFragmentProgramParameters();
  GZ_ASSERT(!params.isNull(), "Null Ogre3D material GPU parameters");

  // Set parameters
  for (auto& p : m_param_map)
  {
    const Ogre::GpuConstantDefinition* const_def = params->_findNamedConstantDefinition(p.first);
    if(const_def)
    {
      params->setNamedConstant(p.first, p.second.m_value);
    }
  }

  const Ogre::GpuConstantDefinition* offsets_def = params->_findNamedConstantDefinition("offsets");
  if(offsets_def)
  {
    // The noise shader contains a simple pseudorandom number generator based on
    // fragment position that is used 3 times for each fragment. Generate 3
    // random numbers here that will be added the inputs to that generator,
    // making the final random numbers different for each frame.
    Ogre::Vector3 offsets(double(rand()) / double(RAND_MAX),
                          double(rand()) / double(RAND_MAX),
                          double(rand()) / double(RAND_MAX));
    params->setNamedConstant("offsets", offsets);
  }
}
