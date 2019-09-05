/*******************************************************************************
 * Copyright (c) 2018 United States Government as represented by the 
 * Administrator of the National Aeronautics and Space Administration. 
 * All rights reserved.
 ******************************************************************************/
#include "LinkTracksPlugin.h"

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/common/Image.hh>


using namespace std;
using namespace gazebo;
using namespace Ogre;

GZ_REGISTER_VISUAL_PLUGIN(LinkTracksPlugin)

/**
 * ctor
 */
LinkTracksPlugin::LinkTracksPlugin() :
  VisualPlugin(),
  mTexWidth(0),
  mTexHeight(0),
  mTextureName("wheelTracks"),
  mMinDistThresh(0.05),
  mTrackWidth(4.5),
  mTrackDepth(1.0),
  mTrackExp(2.0)
{
  mTexture.setNull();

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  mNodeHandle.reset(new ros::NodeHandle("gazebo_client"));
}

/**
 * dtor
 */
LinkTracksPlugin::~LinkTracksPlugin()
{
}

/**
 * Load values from SDF and setup subscriptions
 */
void LinkTracksPlugin::Load(rendering::VisualPtr _sensor, sdf::ElementPtr _sdf)
{
  //-- specify link names
  for(int i = 0; i < 16; i++) {
    char buf[128];
    snprintf(buf, 128, "link_name_%d", i);
    if(_sdf->HasElement(buf)) {
      mLinkName.push_back(_sdf->Get<std::string>(buf));
    }
    else {
      break; // names must be consecutive starting at 0
    }
  }
  // initialize LinkPos and LinkEnable variables
  for(auto& name : mLinkName) {
    mLinkPos.push_back(Vector3(0.0, 0.0, 0.0));
    mLinkEnabled.push_back(1);
  }
  gzlog << "LinkTracksPlugin::Load - " << mLinkName.size() << " links were specified." << endl;
  
  //-- texture name in terrain shader 
  if (_sdf->HasElement("texture_name")) {
    mTextureName = _sdf->Get<string>("texture_name");
  }
  
  //-- initial image load
  if (_sdf->HasElement("load_image")) {
    mLoadImage = _sdf->Get<string>("load_image");
  }
  // if parameter exists on ROS param server, use that instead
  const char* param_name = "/gazebo/plugins/link_tracks/load_image";
  if(ros::param::has(param_name)) {
    ros::param::get(param_name, mLoadImage);
  }
  
  //-- track width
  if (_sdf->HasElement("track_width")) {
    mTrackWidth = _sdf->Get<double>("track_width");
    mTrackWidth = max(mTrackWidth, 0.0);
  }

  //-- track depth
  if (_sdf->HasElement("track_depth")) {
    mTrackDepth = _sdf->Get<double>("track_depth");
    mTrackDepth = max(mTrackDepth, 0.0);
    mTrackDepth = min(mTrackDepth, 1.0);
  }

  //-- track exponent
  if (_sdf->HasElement("track_exponent")) {
    mTrackExp = _sdf->Get<double>("track_exponent");
    mTrackExp = max(mTrackExp, 0.0);
  }
  
  //-- min distance threshold
  if (_sdf->HasElement("min_dist_thresh")) {
    mMinDistThresh = _sdf->Get<double>("min_dist_thresh");
    mMinDistThresh = max(mTrackExp, 0.001);
  }
  
  //== subscribe to ROS messages ==============
  mSaveImageSub = mNodeHandle->subscribe("/gazebo/plugins/link_tracks/save_image", 1,
                                         &LinkTracksPlugin::OnSaveImage, this);
  
  mLinkEnableSub = mNodeHandle->subscribe("/gazebo/plugins/link_tracks/link_enable", 1,
                                          &LinkTracksPlugin::OnLinkEnable, this);

  mDrawEnableSub = mNodeHandle->subscribe("/gazebo/plugins/link_tracks/draw_enable", 1,
                                          &LinkTracksPlugin::OnDrawEnable, this);

  //== Listen to frame update event ===========
  this->mUpdateConnection = event::Events::ConnectPreRender(
    boost::bind(&LinkTracksPlugin::OnUpdate, this));
}

/**
 * Create texture to draw into 
 * @returns true if texture already exists or has successfully been created
 */
bool LinkTracksPlugin::InitTexture()
{
  if (!mTexture.isNull()) {
    // Texture has already been created
    return true;
  }

  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    gzerr << "LinkTracksPlugin::InitTexture - scene pointer is NULL" << endl;
    return false;
  }

  rendering::Heightmap* heightmap = scene->GetHeightmap();
  if (!heightmap) {
    gzerr << "LinkTracksPlugin::InitTexture - heightmap pointer is NULL" << endl;
    return false;
  }

  // This is the parent material--not the terrain materials to which we will
  // assign the new link tracks texture.
  MaterialPtr parentMat = MaterialManager::getSingleton().getByName(heightmap->MaterialName());
  if (parentMat.isNull()) {
    gzerr << "LinkTracksPlugin::InitTexture - material pointer is NULL" << endl;
    return false;
  }
  // Get dimensions of terrain.
  TextureUnitState* normalsTus = parentMat->getTechnique(0)->getPass(0)->getTextureUnitState("normals");
  if (!normalsTus) {
    gzerr << "LinkTracksPlugin::InitTexture - normals TextureUnitState is NULL" << endl;
    return false;
  }
  pair< size_t, size_t > size = normalsTus->getTextureDimensions();
  mTexWidth = size.first;
  mTexHeight = size.second;

  // Create a single link track texture for all terrain materials.
  mTexture = TextureManager::getSingleton().createManual("linkTracksTexture",
    ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, TEX_TYPE_2D,
    mTexWidth, mTexHeight, 0, PF_L8, TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  // Lock the pixel buffer
  HardwarePixelBufferSharedPtr pixelBuffer = mTexture->getBuffer();
  pixelBuffer->lock(HardwareBuffer::HBL_DISCARD);

  const PixelBox& pixelBox = pixelBuffer->getCurrentLock();
  uint8* pixels = static_cast<uint8*>(pixelBox.data);

  // Paint it white
  memset(pixels, 255, mTexWidth * mTexHeight);

  // Unlock the pixel buffer
  pixelBuffer->unlock();

  if (!mLoadImage.empty()) {
    // Load a saved image
    LoadImage(mLoadImage);
    mLoadImage = "";
  }

  gzlog << "LinkTracksPlugin::InitTexture - texture was created." << endl;

  // Assign texture to all cloned terrain materials
  TerrainGroup::TerrainIterator ti = heightmap->OgreTerrain()->getTerrainIterator();
  while (ti.hasMoreElements()) {
    Terrain *terrain = ti.getNext()->instance;
    if (!terrain) {
      gzerr << "LinkTracksPlugin::InitTexture - this should never happen" << endl;
      continue;
    }
    MaterialPtr terrainMat = terrain->getMaterial();
    TextureUnitState* tus =
      terrainMat->getTechnique(0)->getPass(0)->getTextureUnitState(mTextureName);
    if(tus != NULL) {
      tus->setTexture(mTexture);
    }
    else {
      gzerr << "LinkTracksPlugin::InitTexture -ERROR- could not get texture \"" << 
        mTextureName << "\" from terrain shader." << endl;
      return false; 
    }
  }

  return true;
}

/**
 * frame callback
 */
void LinkTracksPlugin::OnUpdate()
{
  // Cannot do this in constructor or Load function because Heightmap is not yet available.
  if (!InitTexture()) {
     return;
  }

  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    gzerr << "LinkTracksPlugin::OnUpdate - scene pointer is NULL" << endl;
    return;
  }

  if(mDrawEnabled) {
    for(int i = 0; i < mLinkName.size(); i++) {
      if(mLinkEnabled[i]) {
        ProcessLink(scene->GetVisual(mLinkName[i]), i);
      }
    }
  }

  if (!mSaveImage.empty()) {
    SaveImage(mSaveImage);
    mSaveImage = "";
  }
}

//limit number of warnings printed to console if we don't have a compatible robot
static const int WARN_MAX_ProcessLink = 40;
static       int warn_cnt_ProcessLink = 0;

/**
 * Get current link position and draw into texture
 */
void LinkTracksPlugin::ProcessLink(const rendering::VisualPtr& visual, const int linkIndex)
{
  if(!visual) {
    if(warn_cnt_ProcessLink++ < WARN_MAX_ProcessLink)
      gzerr << "LinkTracksPlugin::ProcessLink - visual pointer for link " << linkIndex << " is NULL" << endl;
    return;
  }

  ignition::math::Pose3d pose = visual->WorldPose();

  // If link has moved beyond a certain threshold, draw an indentation in terrain.
  // only populate X and Y, we don't care about movement in Z
  Vector3 position(pose.Pos().X(), pose.Pos().Y(), 0.0);
  Vector3 diff = position - mLinkPos[linkIndex];
  if (diff.length() >= mMinDistThresh) {
    mLinkPos[linkIndex] = position;
    Vector2 uv;
    if (TransformLinkPositionToUV(position, uv)) {
      Draw(uv);
    }
  }
}

/**
 * map from X,Y world space into UV
 */
bool LinkTracksPlugin::TransformLinkPositionToUV(const Vector3& wsPosition, Vector2& uv)
{
  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    gzerr << "LinkTracksPlugin::Draw - scene pointer is NULL" << endl;
    return false;
  }

  rendering::Heightmap* heightmap = scene->GetHeightmap();
  if (!heightmap) {
    gzerr << "LinkTracksPlugin::Draw - heightmap pointer is NULL" << endl;
    return false;
  }

  TerrainGroup* terrainGroup = heightmap->OgreTerrain();
  if (!terrainGroup) {
    gzerr << "LinkTracksPlugin::Draw - terrainGroup pointer is NULL" << endl;
    return false;
  }

  long xIndex, yIndex;
  terrainGroup->convertWorldPositionToTerrainSlot(wsPosition, &xIndex, &yIndex);
  Terrain* terrain = terrainGroup->getTerrain(xIndex, yIndex);
  if (!terrain) {
    // This can happen if the rover drives off the edge of the terrain
    return false;
  }

  Vector3 terrainSpacePosition;
  terrain->getTerrainPosition(wsPosition, &terrainSpacePosition);
  const unsigned int tilecount = sqrt(heightmap->TerrainSubdivisionCount());
  uv = Vector2((double(xIndex) + terrainSpacePosition.x) / double(tilecount),
    (double((tilecount - 1) - yIndex) + (1.0f - terrainSpacePosition.y)) / double(tilecount));

  return true;
}

/**
 * draw dot into texture
 */
void LinkTracksPlugin::Draw(const Vector2& uv)
{
  const int halfSize = (mTrackWidth + 2.0) * 0.5;
  const int boxSize = halfSize * 2;

  const double x_real = uv[0] * mTexWidth + 0.5;
  const double y_real = uv[1] * mTexHeight + 0.5;
  // texel origin of link indentation
  const int x = int(x_real);
  const int y = int(y_real);
  // fractional part of texel positions
  const double x_frac = x_real - double(x);
  const double y_frac = y_real - double(y);

  // Indentation is off edge of texture, so don't draw it
  if(x - halfSize < 0 || x + halfSize >= mTexWidth
    || y - halfSize < 0 || y + halfSize >= mTexHeight)
  {
    return;
  }

  Image::Box box(x - halfSize, y - halfSize, x + halfSize, y + halfSize);
  HardwarePixelBufferSharedPtr pixelBuffer = mTexture->getBuffer();

  // Locking a specific box results in glTexSubImage2D being used instead of
  // updating the whole image.
  // Using HBL_NORMAL allows texel data to be read instead of just written. This
  // hurts performance but is necessary for blending with existing texels. Could
  // maybe improve performance by storing copy of texture in RAM and blending
  // with it while using HBL_WRITE_ONLY.
  const PixelBox& pixelBox = pixelBuffer->lock(box, HardwareBuffer::HBL_NORMAL);
  uint8* pixels = static_cast<uint8*>(pixelBox.data);

  const double PIo2 = M_PI / 2.0;

  // Make dot a little smaller than box 
  const double dentRadius = mTrackWidth * 0.5;

  // Paint a spherical indentation
  for (int j = 0; j < boxSize; j++)
  {
    const double texY = double(j - halfSize) + (1.0 - y_frac);
    for(int i = 0; i < boxSize; i++)
    {
      const double texX = double(i - halfSize) + (1.0 - x_frac);
      const double texRadius = sqrt(texX * texX + texY * texY);
      const double depth = min((PIo2 - acos(min(texRadius / dentRadius, 1.0))) / PIo2, 1.0);
      const uint8 oldColor = *pixels;
      const uint8 newColor = uint8((pow(depth, mTrackExp) * mTrackDepth
                                    + (1.0 - mTrackDepth))
                                    * 255.999);
      *pixels++ = min(oldColor, newColor);
    }
 
    pixels += pixelBox.getRowSkip() * PixelUtil::getNumElemBytes(pixelBox.format);
  }

  // Unlock the pixel buffer
  pixelBuffer->unlock();
}

/**
 * ROS message callback
 */
void LinkTracksPlugin::OnSaveImage(const std_msgs::StringConstPtr& msg)
{
  mSaveImage = msg->data;
}

/**
 * ROS message callback
 */
void LinkTracksPlugin::OnLinkEnable(const std_msgs::UInt8MultiArrayConstPtr& msg)
{
  for(int i = 0; i < msg->data.size(); i++) {
    if(i < mLinkEnabled.size()) {
      mLinkEnabled[i] = msg->data[i];
    }
  }
}

/**
 * ROS message callback
 */
void LinkTracksPlugin::OnDrawEnable(const std_msgs::BoolConstPtr& msg)
{
  mDrawEnabled = msg->data;
}

/**
 * 
 */
void LinkTracksPlugin::SaveImage(const string& filename)
{
  if (filename.empty()) {
    gzerr << "LinkTracksPlugin::SaveImage - filename empty" << endl;
    return;
  }

  if (mTexture.isNull()) {
    gzerr << "LinkTracksPlugin::SaveImage - texture has not been created" << endl;
    return;
  }

  size_t index = filename.find_last_of('.');
  if (index == string::npos) {
    gzerr << "LinkTracksPlugin::SaveImage - filename must have .png extension (e.g. /tmp/tracks.png)" << endl;
    return;
  }

  string extension = filename.substr(index+1);
  if (strcasecmp(extension.c_str(), "png") != 0) {
    gzerr << "LinkTracksPlugin::SaveImage - filename must have .png extension (e.g. /tmp/tracks.png)" << endl;
    return;
  }

  // Lock the pixel buffer
  HardwarePixelBufferSharedPtr pixelBuffer = mTexture->getBuffer();
  pixelBuffer->lock(HardwareBuffer::HBL_READ_ONLY);

  const PixelBox& pixelBox = pixelBuffer->getCurrentLock();
  const unsigned char* pixels = static_cast<const unsigned char*>(pixelBox.data);

  // Get pixel data
  common::Image image;
  image.SetFromData(pixels, mTexWidth, mTexHeight, common::Image::L_INT8);

  // Unlock the pixel buffer
  pixelBuffer->unlock();

  // Save it
  image.SavePNG(filename);

  struct stat stat_buf;
  if (stat (filename.c_str(), &stat_buf) != 0) {
    gzerr << "LinkTracksPlugin::SaveImage - failed to write " << filename << endl;
    return;
  }

  gzmsg << "LinkTracksPlugin - saved image to " << filename << endl;
}

/**
 * 
 */
void LinkTracksPlugin::LoadImage(const string& filename)
{
  if (filename.empty()) {
    gzerr << "LinkTracksPlugin::LoadImage - filename empty" << endl;
    return;
  }

  if (mTexture.isNull()) {
    gzerr << "LinkTracksPlugin::LoadImage - texture has not been created" << endl;
    return;
  }

  ifstream ifs(filename.c_str(), ios::binary|ios::in);
  if (!ifs.is_open()) {
    gzerr << "LinkTracksPlugin::LoadImage - could not open " << filename << endl;
    return;
  }

  size_t index = filename.find_last_of('.');
  if (index != string::npos) {
    string extension = filename.substr(index+1);
    DataStreamPtr data_stream(new FileStreamDataStream(filename, &ifs, false));
    Image image;
    image.load(data_stream, extension);
    // FYI, this will scale the image to the texture size. But under normal use
    // we will always be providing an image of the same size as the texture.
    mTexture->loadImage(image);

    gzmsg << "LinkTracksPlugin - loaded image " << filename << endl;
  }

  ifs.close();
}


