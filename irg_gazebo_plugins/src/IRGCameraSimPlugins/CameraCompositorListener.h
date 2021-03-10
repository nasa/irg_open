// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef CameraCompositorListener_h
#define CameraCompositorListener_h

#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <gazebo/rendering/ogre_gazebo.h>
#include <sdf/Element.hh>


namespace irg {

/**
 * @brief This class updates camera sim shader params before rendering.
 * Param (GLSL uniform) initial values are collected from a plugin declaration
 * in your SDF code. These params can also be modified by publishing a ROS
 * message to the appropriate camera_sim topic. notifyMaterialRender() applies
 * these params to the camera sim's compositor's shaders before rendering.
 *
 * Ogre uses "parameter" for shader uniforms, while ROS uses "parameter" for
 * node attributes. In this plugin we use "param" for shader uniforms and
 * "rosparam" for ROS parameters.
 */
class CameraCompositorListener : public Ogre::CompositorInstance::Listener
{
public:
  CameraCompositorListener(sdf::ElementPtr sdf);

  /// \brief Callback that Ogre3D will call before rendering each frame
  virtual void notifyMaterialRender(unsigned int pass_id, Ogre::MaterialPtr& mat) override;

private:
  /// \brief For initializing all data and a subscriber associated with a param
  void initParam(sdf::ElementPtr sdf, const std::string& name, double initial_value);

  /// \brief Subscriber callback that allows user to set a param
  void onParamUpdate(const std_msgs::msg::Float64::SharedPtr msg, const std::string& name);

  // Prefer gazebo_ros::Node over rclcpp::Node because spin() is called by
  // gazebo_ros so you don't have to. Somebody must call spin or callback
  // functions will never get called.
  gazebo_ros::Node::SharedPtr m_node;

  /// Handle to ROS parameters callback
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_rosparam_callback;

  std::string m_topic_uid;

  // Storage for values for Ogre GpuProgramParameters (GLSL uniforms)
  struct ParamData
  {
    Ogre::Real m_value;
    rclcpp::Subscription<std_msgs::msg::Float64>::ConstSharedPtr m_subscriber;
  };
  std::map<std::string, ParamData> m_param_map;
};

}


#endif // CameraCompositorListener_h
