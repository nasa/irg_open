/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#ifndef LinkTracksPlugin_h
#define LinkTracksPlugin_h

#include <gazebo/common/Plugin.hh>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
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
    std::vector<uint8_t>        mLinkEnabled; //< disable draw of individual link
    
    bool   mDrawEnabled; //< disable all drawing 
    
    double mMinDistThresh;
    double mTrackWidth;
    double mTrackDepth;
    double mTrackExp;
    
    // For subscribing to ROS messages
    rclcpp::Node::SharedPtr mNodeHandle;
    rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr mSaveImageSub;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::ConstSharedPtr mLinkEnableSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr mDrawEnableSub;
    
    std::string mSaveImage;
    std::string mLoadImage;
  };

}

#endif // LinkTracksPlugin_h

