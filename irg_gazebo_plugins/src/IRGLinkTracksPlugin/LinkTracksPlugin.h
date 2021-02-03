// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef LinkTracksPlugin_h
#define LinkTracksPlugin_h

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
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

    virtual void Load(rendering::VisualPtr _sensor, sdf::ElementPtr _sdf);

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
    void OnSaveImage(const std_msgs::StringConstPtr& msg);
    void OnLinkEnable(const std_msgs::UInt8MultiArrayConstPtr& msg);
    void OnDrawEnable(const std_msgs::BoolConstPtr& msg);

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
    
    std::vector<ros::Publisher> mAltitudePublisher;
    std::vector<double>         mAltitudeThreshold;

    double mMinDistThresh;
    double mTrackWidth;
    double mTrackDepth;
    double mTrackExp;
    
    // For subscribing to ROS messages
    std::unique_ptr<ros::NodeHandle> mNodeHandle;
    ros::Subscriber   mSaveImageSub;
    ros::Subscriber   mLinkEnableSub;
    ros::Subscriber   mDrawEnableSub;
    
    std::string mSaveImage;
    std::string mLoadImage;
  };

}

#endif // LinkTracksPlugin_h

