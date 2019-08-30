/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef LinkTracksPlugin_h
#define LinkTracksPlugin_h

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreTexture.h>

namespace gazebo {

  class LinkTracksPlugin : public VisualPlugin
  {
  public: 
    LinkTracksPlugin();
    ~LinkTracksPlugin();

    virtual void Load(rendering::VisualPtr _sensor, sdf::ElementPtr _sdf);

    bool InitTexture();

    void OnUpdate();

    void ProcessLink(const rendering::VisualPtr& visual, const int linkIndex);

    // Pass link position in world space, get position in texture space
    // Returns true if position is within texture.
    bool TransformLinkPositionToUV(const Ogre::Vector3& wsPosition, Ogre::Vector2& uv);

    void Draw(const Ogre::Vector2& uv);

    void OnSaveImage(const std_msgs::StringConstPtr& msg);

    void SaveImage(const std::string& filename);

    void LoadImage(const std::string& filename);

  private:
    // Connection to the update event
    event::ConnectionPtr mUpdateConnection;

    int mTexWidth;
    int mTexHeight;

    std::string      mTextureName;
    Ogre::TexturePtr mTexture;

    Ogre::Vector3 mLinkPos[4];

    double mTrackWidth;
    double mTrackDepth;
    double mTrackExp;
    
    // For subscribing to ROS messages
    std::unique_ptr<ros::NodeHandle> mNodeHandle;
    ros::Subscriber mSaveImageSub;
    std::string mSaveImage;
    std::string mLoadImage;
  };

}

#endif // LinkTracksPlugin_h

