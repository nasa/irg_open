// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "CameraCompositorListener.h"
#include <gazebo/common/Assert.hh>


using namespace irg;


CameraCompositorListener::CameraCompositorListener(sdf::ElementPtr sdf)
{
  m_node_handle.reset(new ros::NodeHandle("irg_camera_compositor_listener"));

  // Get UID for topic from the sdf element
  if (sdf->HasElement("topic_uid")) {
    m_topic_uid = sdf->Get<std::string>("topic_uid");

    // Remove all single- and double-quotes, just in case the user used any.
    m_topic_uid.erase(remove( m_topic_uid.begin(), m_topic_uid.end(), '\'' ), m_topic_uid.end());
    m_topic_uid.erase(remove( m_topic_uid.begin(), m_topic_uid.end(), '\"' ), m_topic_uid.end());
  }

  // Declare shader param names and default values
  initParam("exposure", 1.0);
  initParam("energy_conversion", 1.0);
  initParam("read_noise_std_dev", 0.8);
  initParam("shot_noise_coeff", 0.3);
  initParam("gain", 1.0);
  initParam("gamma", 1.0);
  initParam("adc_bits", 12.0);

  // Get new values from the sdf element
  for (auto& p : m_param_map)
  {
    if (sdf->HasElement(p.first)) {
      p.second.m_value = sdf->Get<Ogre::Real>(p.first);
    }
  }
}

void CameraCompositorListener::initParam(std::string name, double initial_value)
{
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
  // The main gotchas are a) you must use boost::function, not std::function or
  // auto, and b) you must pass your topic type as reference (don't forget the
  // ampersand).
  // Commented out are examples of how you can also use std::bind or a lambda
  // to achieve the same result. I'm using boost::bind simply to be consistent
  // with boost::function.
  boost::function<void (const std_msgs::Float64::ConstPtr& msg)> func =
    boost::bind(&CameraCompositorListener::onParamUpdate, this, _1, name);
    //std::bind(&CameraCompositorListener::onParamUpdate, this, std::placeholders::_1, name);
    //[this, name](const std_msgs::Float64::ConstPtr& msg){ m_param_map[name].m_value = msg->data; };

  // Subscribe using our fancy bound function pointer.
  m_param_map[name].m_subscriber = m_node_handle->subscribe(topic, 1, func);
}

void CameraCompositorListener::onParamUpdate(const std_msgs::Float64::ConstPtr& msg, std::string name)
{
  m_param_map[name].m_value = msg->data;
}

void CameraCompositorListener::notifyMaterialRender(unsigned int pass_id,
                                                    Ogre::MaterialPtr& mat)
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
