// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "CelestialBodyPlugin.h"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>

using namespace irg;
using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CelestialBodyPlugin)

CelestialBodyPlugin::CelestialBodyPlugin() :
  ModelPlugin(),
  m_transformListener(m_tfBuffer)
{
  m_timer.Start();
}

CelestialBodyPlugin::~CelestialBodyPlugin()
{
}


void CelestialBodyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  m_model = _model;

  if (!_sdf->HasElement("frame")) {
    gzerr << "CelestialBodyPlugin: you must specify a frame element." << endl;
    return;
  }
  m_frame = _sdf->Get<string>("frame");

  if (!_sdf->HasElement("radius")) {
    gzerr << "CelestialBodyPlugin: you must specify a radius." << endl;
    return;
  }
  m_radius = _sdf->Get<double>("radius");

  if (!_sdf->HasElement("render_distance")) {
    gzerr << "CelestialBodyPlugin: you must specify a render_distance." << endl;
    return;
  }
  m_renderDistance = _sdf->Get<double>("render_distance");

  m_lightSource = false;
  if (_sdf->HasElement("light_source")) {
    m_lightSource = _sdf->Get<bool>("light_source");
  }

  // Listen to the update event. This event is broadcast every sim iteration.
  // If result goes out of scope. updates will stop, so it assigned to a member variable.
  m_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CelestialBodyPlugin::OnUpdate, this));
}

void CelestialBodyPlugin::OnUpdate()
{
  // Only continue if a second has elapsed
  if (m_timer.GetElapsed().Double() < 1.0)
  {
    return;
  }
  m_timer.Reset();
  m_timer.Start();

  // Get most recent transform
  geometry_msgs::TransformStamped sXform;
  try
  {
    sXform = m_tfBuffer.lookupTransform("celestial_body_origin", m_frame, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("CelestialBodyPlugin::OnUpdate - %s", ex.what());
    return;
  }

  geometry_msgs::Vector3& v = sXform.transform.translation;
  if (v.x == 0.0 && v.y == 0.0 && v.z == 0.0)
  {
    gzerr << "CelestialBodyPlugin: tf offset from celestial_body_origin to "
          << m_frame << " is all zeros. Something is broken." << endl;
    // Using all zeros will likely crash Ogre3D, so we return here in hopes that
    // a better transform will be available later.
    return;
  }

  ignition::math::Vector3d pos(v.x, v.y, v.z);
  const double distance = pos.Length();
  pos.Normalize();
  pos *= m_renderDistance;

  if(m_lightSource)
  {
    // If this is the sun, set the rotation to describe the accompanying
    // directional light's direction. This assumes that light will have its
    // direction set to (1, 0, 0) in the .sdf file.
    const double azimuth = atan2(-pos[1], -pos[0]);
    const double zenith = atan2(pos[2], sqrt(pos[0] * pos[0] + pos[1] * pos[1]));
    m_model->SetWorldPose(ignition::math::Pose3d(pos[0], pos[1], pos[2], 0, zenith, azimuth));
  }
  else
  {
    // Otherwise, set the rotation as expected, so any texture map on the body
    // will be properly oriented.
    geometry_msgs::Quaternion& rot = sXform.transform.rotation;
    ignition::math::Quaterniond quat(rot.w, rot.x, rot.y, rot.z);
    m_model->SetWorldPose(ignition::math::Pose3d(pos, quat));
  }

  const double scale = m_radius * m_renderDistance / distance;
  m_model->SetScale(ignition::math::Vector3d(scale, scale, scale), true);
}

