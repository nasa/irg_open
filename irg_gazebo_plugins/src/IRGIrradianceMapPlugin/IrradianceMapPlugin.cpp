// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "IrradianceMapPlugin.h"

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Heightmap.hh>
#include <gazebo/common/Image.hh>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/RenderSystems/GL/OgreGLTexture.h>


using namespace irg;
using namespace std;
using namespace gazebo;
using namespace Ogre;

GZ_REGISTER_VISUAL_PLUGIN(IrradianceMapPlugin)

int IrradianceMapPlugin::m_index_counter = 0;

IrradianceMapPlugin::IrradianceMapPlugin() :
  VisualPlugin()
{
  // Initialize glGenerateMipmap function address
  glewInit();

  m_unique_index = m_index_counter ++;

  m_texture.setNull();

  m_use_visibility_bitmask = false;

  // Lux from starlight is 0.00022 according to
  // https://www.researchgate.net/publication/238589855_Night_illumination_in_the_visible_NIR_and_SWIR_spectral_bands
  m_background_color = ColourValue(0.00022f, 0.00022f, 0.00022f, 1.0f);

  // Initialize ros, if it has not already been initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
  }

  m_timer.Start();
}

IrradianceMapPlugin::~IrradianceMapPlugin()
{
}

void IrradianceMapPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr element)
{
  if (!element->HasElement("texture_unit"))
  {
    gzerr << "IrradianceMapPlugin::Load: you must specify a texture_unit_state element." << endl;
    return;
  }
  m_texture_unit_name = element->Get<string>("texture_unit");

  if (element->HasElement("visibility_bitmask"))
  {
    m_use_visibility_bitmask = true;
    // Cannot read a hexidecimal value of the form 0x******** using element->Get().
    // It always results in 0. Must instead use std::stoul().
    const string mask_str = element->Get<string>("visibility_bitmask");
    m_visibility_bitmask = std::stoul(mask_str, nullptr, 16);
  }

  if (element->HasElement("background_color"))
  {
    ignition::math::Vector3d color = element->GetElement("background_color")->Get<ignition::math::Vector3d>();
    m_background_color = ColourValue(color[0], color[1], color[2], 1.0f);
  }

  // Listen to the update event. This event is broadcast every sim iteration.
  this->m_update_connection = event::Events::ConnectPreRender(
    boost::bind(&IrradianceMapPlugin::onUpdate, this));

  rendering::ScenePtr scene = rendering::get_scene();
  if (!scene)
  {
    gzerr << "IrradianceMapPlugin::Load: scene pointer is NULL" << endl;
    return;
  }

  String source_cubemap_name("SourceEnvironmentCubemap" + StringConverter::toString(m_unique_index));
  const int size = 512;
  const int numMipMaps = log2(size);
  m_texture = TextureManager::getSingleton().createManual(
        source_cubemap_name, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        TEX_TYPE_CUBE_MAP, size, size, numMipMaps, PF_FLOAT32_RGB,
        TU_DYNAMIC_WRITE_ONLY | TU_RENDERTARGET | TU_AUTOMIPMAP);

  ignition::math::Pose3d pose = visual->WorldPose();
  for (int i = 0; i < 6; i++)
  {
    // SceneManager destructor is responsible for deleting Cameras
    SceneManager* scene_manager = scene->OgreSceneManager();
    m_cameras[i] = scene_manager->createCamera("CameraCubeMap" + StringConverter::toString(i));
    m_cameras[i]->setFOVy(Radian(Math::PI / 2));
    m_cameras[i]->setAspectRatio(1);
    m_cameras[i]->setNearClipDistance(0.1);
    m_cameras[i]->setFarClipDistance(200000);
    m_cameras[i]->setPosition(pose.Pos()[0], pose.Pos()[1], pose.Pos()[2]);

    // OpenGL cubemaps are arranged using RenderMan's left-handed coordinate
    // system resulting in the entire map being mirrored when rendered looking
    // out from the center like this, so we must somehow mirror the entire
    // cubemap to use it in the scene properly.
    // https://stackoverflow.com/questions/11685608/convention-of-faces-in-opengl-cubemapping
    // http://marcinignac.com/blog/pragmatic-pbr-hdr
    // This problem might be solved in Ogre3D 2.x, but I'm not certain.
    // https://bitbucket.org/sinbad/ogre/pull-requests/650/fixed-cubemap-reflections-so-there-are/activity
    // Since we're using Ogre3D 1.9, we will solve this problem by rendering
    // our cubemaps like below and then negating the y-component of cube lookups
    // in our shaders.
    switch(i)
    {
    case 0: // right
      m_cameras[i]->yaw( Radian( -Math::PI / 2) );
      m_cameras[i]->roll( Radian( -Math::PI / 2) );
      break;
    case 1: // left
      m_cameras[i]->yaw( Radian( Math::PI / 2) );
      m_cameras[i]->roll( Radian( Math::PI / 2) );
      break;
    case 2: // up
      m_cameras[i]->pitch( Radian( Math::PI ) );
      break;
    case 3: // down
      break;
    case 4: // front
      m_cameras[i]->pitch( Radian( Math::PI / 2 ) );
      break;
    case 5: // back
      m_cameras[i]->pitch( Radian( -Math::PI / 2 ) );
      m_cameras[i]->roll( Radian( Math::PI ) );
      break;
    }

    RenderTarget* renderTarget = m_texture->getBuffer(i)->getRenderTarget();
    // RenderTarget destructor is responsible for deleting Viewports
    m_viewports[i] = renderTarget->addViewport(m_cameras[i]);
    m_viewports[i]->setOverlaysEnabled(false);
    m_viewports[i]->setClearEveryFrame(true);
    m_viewports[i]->setBackgroundColour(m_background_color);
  }

  m_cubemap_filter.reset(new CubemapFilter(m_unique_index, source_cubemap_name));

  gzlog << "IrradianceMapPlugin::Load: complete." << endl;
}

void IrradianceMapPlugin::onUpdate()
{
  // Only continue if a second has elapsed
  if (m_timer.GetElapsed().Double() < 2.0)
  {
    return;
  }
  m_timer.Reset();
  m_timer.Start();

  // Bail out if plugin is not initialized
  if (m_texture.isNull())
  {
    return;
  }

  rendering::ScenePtr scene = rendering::get_scene();
  uint32_t original_visibility_bitmask;
  if(m_use_visibility_bitmask && scene)
  {
    // Use a user-defined visibility mask to limit what is rendered into the
    // irradiance map.
    SceneManager* scene_manager = scene->OgreSceneManager();
    original_visibility_bitmask = scene_manager->getVisibilityMask();
    scene_manager->setVisibilityMask(m_visibility_bitmask);
  }

  for (int i = 0; i < 6; i++)
  {
    m_cameras[i]->_renderScene(m_viewports[i], false);
  }

  if(m_use_visibility_bitmask && scene)
  {
    // Restore original visibility mask
    scene->OgreSceneManager()->setVisibilityMask(original_visibility_bitmask);
  }

  // Ogre 1.9 does not appear to generate mipmaps for dynamic textures. It uses
  // GL_GENERATE_MIPMAP to do it automatically, but that was deprecated. So we
  // make the appropriate call to generate them here.
  GLTexture* gltex = static_cast<GLTexture*>(m_texture.getPointer());
  glBindTexture(gltex->getGLTextureTarget(), gltex->getGLID());
  glGenerateMipmap(gltex->getGLTextureTarget());

  // Filter source texture to create irradiance map
  m_cubemap_filter->render();

  // Assign our dynamic texture wherever it is required.
  // Ogre materials are all created before a visual plugin constructor is
  // called, so a named texture created in such a plugin cannot be referenced by
  // any Ogre material. The solution here is to search for TextureUnitStates
  // with the right name and assign our dynamic texture.
  // I also tried using world and model plugins, but their prerender callbacks
  // are never called. A system plugin is probably the wrong choice because it
  // is applied to gzclient and not gzserver and we want to do off-screen
  // rendering with gzserver.
  ResourceManager::ResourceMapIterator res_it = MaterialManager::getSingleton().getResourceIterator();
  while (res_it.hasMoreElements())
  {
    ResourcePtr resource = res_it.getNext();
    MaterialPtr material = resource.staticCast<Material>();
    Material::TechniqueIterator tech_it = material->getTechniqueIterator();
    while (tech_it.hasMoreElements())
    {
      Technique* technique = tech_it.getNext();
      Technique::PassIterator pass_it = technique->getPassIterator();
      while (pass_it.hasMoreElements())
      {
        Pass* pass = pass_it.getNext();
        TextureUnitState* tus = pass->getTextureUnitState(m_texture_unit_name);
        if (tus != 0)
        {
          //tus->setTexture(m_texture);  // source texture
          tus->setTexture(m_cubemap_filter->getTexture());  // irradiance map
        }
      }
    }
  }
}

