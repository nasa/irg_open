// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "LinkTracksPlugin.h"

#include <algorithm>

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/common/Image.hh>

#include <OGRE/Terrain/OgreTerrainGroup.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <GL/gl.h> 

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
  mDrawEnabled(true),
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
  char buf[128];
  std::string param;

  //-- specify link names
  for(int i = 0; i < 16; i++) {
    snprintf(buf, 128, "link_name_%d", i);
    if(_sdf->HasElement(buf)) {
      mLinkName.push_back(_sdf->Get<std::string>(buf));
    }
    else {
      break; // names must be consecutive starting at 0
    }
  }
  // initialize LinkPos and LinkEnable variables
  for(unsigned int i = 0; i < mLinkName.size(); i++) {
    mLinkPos.push_back(Vector3(0.0, 0.0, 0.0));
    mLinkEnabled.push_back(1);
  }
  gzlog << "LinkTracksPlugin::Load - " << mLinkName.size() << " links were specified." << endl;
  
  //-- texture name in terrain shader 
  param = "texture_name";
  if (_sdf->HasElement(param)) {
    mTextureName = _sdf->Get<string>(param);
  }
  
  //-- initial image load
  param = "load_image";
  if (_sdf->HasElement(param)) {
    mLoadImage = _sdf->Get<string>(param);
  }
  // if parameter exists on ROS param server, use that instead
  const char* param_name = "/gazebo/plugins/link_tracks/load_image";
  if(ros::param::has(param_name)) {
    ros::param::get(param_name, mLoadImage);
  }
  
  //-- track width
  param = "track_width";
  if (_sdf->HasElement(param)) {
    mTrackWidth = _sdf->Get<double>(param);
    mTrackWidth = max(mTrackWidth, 0.0);
  }

  //-- track depth
  param = "track_depth";
  if (_sdf->HasElement(param)) {
    mTrackDepth = _sdf->Get<double>(param);
    mTrackDepth = std::max(mTrackDepth, 0.0);
    mTrackDepth = std::min(mTrackDepth, 1.0);
  }

  //-- track exponent
  param = "track_exponent";
  if (_sdf->HasElement(param)) {
    mTrackExp = _sdf->Get<double>(param);
    mTrackExp = std::max(mTrackExp, 0.0);
  }
  
  //-- min distance threshold
  param = "min_dist_thresh";
  if (_sdf->HasElement(param)) {
    mMinDistThresh = _sdf->Get<double>(param);
    mMinDistThresh = std::max(mMinDistThresh, 0.001);
  }
  
  //-- initial draw state
  param = "draw_enabled";
  if (_sdf->HasElement(param)) {
    mDrawEnabled = _sdf->Get<bool>(param);
  }
  
  //-- altitude enable
  param = "altitude_tracking_enabled";
  if (_sdf->HasElement(param)) {
    double defaultThreshold = 999.9;
    param = "altitude_threshold";
    if (_sdf->HasElement(param)) {
      defaultThreshold = _sdf->Get<double>(param);
    }
    int i = 0;
    for(auto& name : mLinkName) {
      // replace all `:` in link name with `_`
      std::string topicLeaf = name;
      std::replace(topicLeaf.begin(), topicLeaf.end(), ':', '_');
      std::string topicName = "/gazebo/plugins/link_tracks/altitude/"+topicLeaf;
      mAltitudePublisher.push_back(mNodeHandle->advertise<geometry_msgs::Vector3Stamped>(topicName, 1));
      double threshold = defaultThreshold;
      snprintf(buf, 128, "altitude_threshold_%d", i);
      if (_sdf->HasElement(buf)) {
        threshold = _sdf->Get<double>(buf);
      }
      mAltitudeThreshold.push_back(threshold);
      i++;
    }
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

static uint32_t MAX_ERRORS        = 50;
static uint32_t sceneErrors       = 0;
static uint32_t heightmapErrors   = 0;
static uint32_t materialErrors    = 0;
static uint32_t normalsTusErrors  = 0;
static uint32_t textureNameErrors = 0;
static uint32_t visualErrors      = 0;

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
    if(sceneErrors++ < MAX_ERRORS) {
      gzerr << "LinkTracksPlugin::InitTexture - scene pointer is NULL" << endl;
    }
    return false;
  }

  rendering::Heightmap* heightmap = scene->GetHeightmap();
  if (!heightmap) {
    if(heightmapErrors++ < MAX_ERRORS) {
      gzerr << "LinkTracksPlugin::InitTexture - heightmap pointer is NULL" << endl;
    }
    return false;
  }

  // This is the parent material--not the terrain materials to which we will
  // assign the new link tracks texture.
  MaterialPtr parentMat = MaterialManager::getSingleton().getByName(heightmap->MaterialName());
  if (parentMat.isNull()) {
    if(materialErrors++ < MAX_ERRORS) {
      gzerr << "LinkTracksPlugin::InitTexture - material pointer is NULL" << endl;
    }
    return false;
  }
  
  // Get dimensions of terrain.
  TextureUnitState* normalsTus = parentMat->getTechnique(0)->getPass(0)->getTextureUnitState("normals");
  if (!normalsTus) {
    if(normalsTusErrors++ < MAX_ERRORS) {
      gzerr << "LinkTracksPlugin::InitTexture - normals TextureUnitState is NULL" << endl;
    }
    return false;
  }
  pair< size_t, size_t > size = normalsTus->getTextureDimensions();
  mTexWidth = size.first;
  mTexHeight = size.second;
  
  // TODO: for some cases we may want the tracks texture to have more detail than 
  // the normal map of the terrain. This code is untested, if we want to enable 
  // it in the future it needs to be checked. 
  if(false) {
    // get max texture size
    GLint glMaxTexSize;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &glMaxTexSize);
    int maxDim   = std::max(mTexWidth, mTexHeight);
    double scale = (double)glMaxTexSize/(double)maxDim;
    mTexWidth    = static_cast<int>(mTexWidth * scale);
    mTexHeight   = static_cast<int>(mTexHeight * scale);
    mTrackWidth  = mTrackWidth * scale; 
  }

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
  if(false) { // checker board pattern for debugging
    const int gridSz = 32;
    for(int r = 0; r < mTexHeight; r++) {
      for(int c = 0; c < mTexWidth; c++) {
        int idx = (r*mTexWidth)+c;
        int rd = r/gridSz;
        int cd = c/gridSz;
        if(rd % 2 == 0 && cd % 2 == 0) {
          pixels[idx] = 255-r%gridSz-c%gridSz;
        }
        else if(rd % 2 == 1 && cd % 2 == 1) {
          pixels[idx] = 255-r%gridSz-c%gridSz;
        }
      }
    }
  }

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
      if(textureNameErrors++ < MAX_ERRORS) {
        gzerr << "LinkTracksPlugin::InitTexture -ERROR- could not get texture \"" << 
          mTextureName << "\" from terrain shader." << endl;
      }
      return false; 
    }
  }

  return true;
}

/**
 * frame callback
 * NOTE: it appears that the PreRender Gazebo event is *not* called before
 * a render event, but instead called at the physics update rate (i.e. 
 * usually around 1000Hz). This is not a problem as long as the distance
 * threshold is set to a reasonable value but something we should keep 
 * an eye on (in this and other plugins).
 */
void LinkTracksPlugin::OnUpdate()
{
  // Cannot do this in constructor or Load function because Heightmap is not yet available.
  if (!InitTexture()) {
     return;
  }

  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene) {
    if(sceneErrors++ < MAX_ERRORS) {
      gzerr << "LinkTracksPlugin::OnUpdate - scene pointer is NULL" << endl;
    }
    return;
  }
  rendering::Heightmap* heightmap = scene->GetHeightmap();
  if (!heightmap) {
    if(heightmapErrors++ < MAX_ERRORS) {
      gzerr << "LinkTracksPlugin::OnUpdate - heightmap pointer is NULL" << endl;
    }
    return;
  }

  for(unsigned int i = 0; i < mLinkName.size(); i++) {
    ProcessLink(scene->GetVisual(mLinkName[i]), i, heightmap);
  }

  if (!mSaveImage.empty()) {
    SaveImage(mSaveImage);
    mSaveImage = "";
  }
}

/**
 * Get current link position and draw into texture
 */
void LinkTracksPlugin::ProcessLink(const rendering::VisualPtr& visual, 
                                   const unsigned int linkIndex, 
                                   const rendering::Heightmap* heightmap)
{
  if(!visual) {
    if(visualErrors++ < MAX_ERRORS) {
      gzerr << "LinkTracksPlugin::ProcessLink - visual pointer for link " 
            << linkIndex << " (" << mLinkName[linkIndex] << ") is NULL" << endl;
    }
    return;
  }

  ignition::math::Pose3d pose = visual->WorldPose();
  
  // check altitude
  if(mAltitudePublisher.size() > linkIndex) {
    double altitude;
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vector.x = pose.Pos().X();
    msg.vector.y = pose.Pos().Y();
    // Gazebo Heightmap does not provide accessors for extents. If x,y is outside
    // the bounds of the Heightmap, the Height() call returns 0
    msg.vector.z = altitude = pose.Pos().Z() - heightmap->Height(msg.vector.x, msg.vector.y);
    mAltitudePublisher[linkIndex].publish(msg);
    mLinkEnabled[linkIndex] = (altitude < mAltitudeThreshold[linkIndex]) ? true : false;
  }

  if(mDrawEnabled && mLinkEnabled[linkIndex]) {
    // If link has moved beyond a certain threshold, draw an indentation in terrain.
    // only populate X and Y, we don't care about movement in Z
    Vector3 position(pose.Pos().X(), pose.Pos().Y(), 0.0);
    Vector3 diff = position - mLinkPos[linkIndex];
    if (diff.length() >= mMinDistThresh) {
      mLinkPos[linkIndex] = position;
      Vector2 uv;
      if (TransformLinkPositionToUV(position, uv, heightmap)) {
        Draw(uv);
      }
    }
  }
}

/**
 * map from X,Y world space into UV
 */
bool LinkTracksPlugin::TransformLinkPositionToUV(const Vector3& wsPosition, 
                                                 Vector2& uv,
                                                 const rendering::Heightmap* heightmap)
{
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
  
  // Workaround for Gazebo reporting 16 on TerrainSubdivisionCount()
  // when there should only be one. 
  int terrainsCount = 0;
  TerrainGroup::TerrainIterator it = terrainGroup->getTerrainIterator();
  while(it.hasMoreElements()) {
    it.moveNext();
    terrainsCount++;
  }
  
  Vector3 terrainSpacePosition;
  terrain->getTerrainPosition(wsPosition, &terrainSpacePosition);
  //const unsigned int tilecount = sqrt(heightmap->TerrainSubdivisionCount());
  const unsigned int tilecount = sqrt(terrainsCount);
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
  for(unsigned int i = 0; i < msg->data.size(); i++) {
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


