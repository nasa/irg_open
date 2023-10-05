// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef LinkTracksPlugin_h
#define LinkTracksPlugin_h

#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreTexture.h>

namespace gazebo {
  
  namespace rendering {
    class Heightmap;
  }

  class LinkTracksPlugin : public VisualPlugin
  {
  public: 
    LinkTracksPlugin();
    ~LinkTracksPlugin();

    virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

    bool InitTexture();

    void OnUpdate();

    void ProcessLink(const rendering::VisualPtr& visual, 
                     const unsigned int linkIndex,
                     const rendering::Heightmap* heightmap);

    // Pass link position in world space, get position in texture space
    // Returns true if position is within texture.
    bool TransformLinkPositionToUV(const Ogre::Vector3& wsPosition, 
                                   Ogre::Vector2& uv, 
                                   const rendering::Heightmap* heightmap);

    void Draw(const Ogre::Vector2& uv);

    void SaveImage(const std::string& filename);

    void LoadImage(const std::string& filename);
    
    //-- ROS callbacks
    void OnSaveImage (const std_msgs::msg::String::SharedPtr          msg);
    void OnLinkEnable(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void OnDrawEnable(const std_msgs::msg::Bool::SharedPtr            msg);

    
  private:
    // Connection to the update event
    event::ConnectionPtr mUpdateConnection;
    
    int mTexWidth;
    int mTexHeight;

    std::string       mTextureName;
    Ogre::TexturePtr  mTexture;

    std::vector<std::string>    mLinkName;
    std::vector<Ogre::Vector3>  mLinkPos;
    std::vector<uint8_t>        mLinkEnabled; //< enable/disable draw of individual link
    
    bool                        mDrawEnabled; //< enable/disable all drawing 
    bool                        mAltitudeEnabled; //< enable/disable tracking of altitude above terrain
    
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr> mAltitudePublisher;
    std::vector<double>         mAltitudeThreshold;

    double mMinDistThresh;
    double mTrackWidth;
    double mTrackDepth;
    double mTrackExp;
    
    // For subscribing to ROS messages
    gazebo_ros::Node::SharedPtr mRosNode;
    rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr mSaveImageSub;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::ConstSharedPtr mLinkEnableSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr mDrawEnableSub;
    
    std::string mSaveImage;
    std::string mLoadImage;
  };

}

#endif // LinkTracksPlugin_h

