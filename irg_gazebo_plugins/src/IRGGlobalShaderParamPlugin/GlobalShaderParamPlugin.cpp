// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "GlobalShaderParamPlugin.h"

#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>

#include <OgreResourceManager.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>

#include <ros/console.h>

using namespace irg;
using namespace std;
using namespace gazebo;
using namespace Ogre;
using namespace irg_gazebo_plugins;

GZ_REGISTER_VISUAL_PLUGIN(GlobalShaderParamPlugin)

#define OGRE_GREATER_THAN_1_8 !(OGRE_VERSION < ((1 << 16) | (9 << 8) | 0))

/**
 * 
 */
//========================================================
GlobalShaderParamPlugin::GlobalShaderParamPlugin() :
  VisualPlugin(),
  m_hasUpdates(false),
  m_cacheCleared(true)
{
  m_nodeHandle.reset(new ros::NodeHandle("gazebo_global_shader_param_plugin"));
  
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
  }
}

GlobalShaderParamPlugin::~GlobalShaderParamPlugin()
{
}

inline int8_t shaderTypeFromString(const std::string& str) {
  if(str.compare("vertex") == 0)
    return ShaderParamUpdate::SHADER_TYPE_VERTEX;
  if(str.compare("fragment") == 0)
    return ShaderParamUpdate::SHADER_TYPE_FRAGMENT;
  gzerr << "GlobalShaderParamPlugin: shader type string \"" << str << "\" is not recognized\n";
  return -1;
}

/**
 * 
 */
void GlobalShaderParamPlugin::Load(rendering::VisualPtr _sensor, sdf::ElementPtr _sdf)
{
  // get param names to cache
  sdf::ElementPtr paramElem = _sdf->GetElement("param");
  while(paramElem) {
    if (!paramElem->HasElement("type") || !paramElem->HasElement("name")) {
      gzerr << "GlobalShaderParamPlugin: <param> must have <type> and <name>" << std::endl;
    }
    else {
      int8_t shaderType = shaderTypeFromString(paramElem->Get<std::string>("type"));
      if(shaderType >= 0 && shaderType < ShaderParamUpdate::NUM_SHADER_TYPES) {
        std::string paramName = paramElem->Get<std::string>("name");
        // add a paramName entry in the params list map
        m_paramsListMap[shaderType][paramName];
      }
    }
    paramElem = paramElem->GetNextElement("param");
  }
    
  std::string topic("/gazebo/global_shader_param");
  m_subscriber = m_nodeHandle->subscribe(topic, 20,
      &GlobalShaderParamPlugin::onShaderParamUpdate, this);

  //-- connection callbacks ---------------
  // This should be onPreRender, but we have found shaders in complex scenes
  // are not all available before prerender. Using onPostRender instead, though
  // this may result in a one-frame delay in some shader updates.
  m_postRenderConnection = event::Events::ConnectPostRender(
      boost::bind(&GlobalShaderParamPlugin::onPostRender, this));

  m_entityAddedConnection = event::Events::ConnectAddEntity(
      boost::bind(&GlobalShaderParamPlugin::clearCache, this));

  m_entityDeletedConnection = event::Events::ConnectDeleteEntity(
      boost::bind(&GlobalShaderParamPlugin::clearCache, this));

  m_worldCreatedConnection = event::Events::ConnectWorldCreated(
      boost::bind(&GlobalShaderParamPlugin::onWorldCreated, this));

}

/**
 * 
 */
void GlobalShaderParamPlugin::onShaderParamUpdate(const irg_gazebo_plugins::ShaderParamUpdate::ConstPtr& msg)
{
  Lock guard(m_mutex);
  const std::string& paramName = msg->paramName;
  const int8_t shaderType = msg->shaderType;
  if(paramName.length() == 0) {
    clearCache();
  }
  else if (shaderType >= 0 && shaderType < ShaderParamUpdate::NUM_SHADER_TYPES){
    // if we've never seen this paramName before, clear the cache
    if(m_paramsListMap[shaderType].find(paramName) == m_paramsListMap[shaderType].end()) {
      clearCache();
    }
    m_paramsListMap[shaderType][paramName]; // ensure key exists
    m_paramUpdateMap[shaderType][paramName] = msg->paramValue;
  }
  else {
    gzerr << "GlobalShaderParamPlugin::onShaderParamUpdate - unknown shaderType: " << shaderType << std::endl;
  }
  m_hasUpdates = true;
}

void GlobalShaderParamPlugin::onAddEntity()
{
  clearCache();
}

void GlobalShaderParamPlugin::onDeleteEntity()
{
  clearCache();
}

void GlobalShaderParamPlugin::onWorldCreated()
{
  clearCache();
}

/**
 * clear out all cached Ogre::GpuProgramParametersSharedPtr lists
 */
void GlobalShaderParamPlugin::clearCache()
{
  Lock guard(m_mutex);
  for(int i = 0; i < ShaderParamUpdate::NUM_SHADER_TYPES; i++) {
    for(auto& pair : m_paramsListMap[i]) { 
        pair.second.clear();
    }
  }
  m_cacheCleared = true;
}

/**
 * 
 */
void GlobalShaderParamPlugin::buildCache() 
{
  for(int shaderType = 0; shaderType < ShaderParamUpdate::NUM_SHADER_TYPES; shaderType++) {
    for(auto& pair : m_paramsListMap[shaderType]) {
      cacheParams(shaderType, pair.first, pair.second);
    }
  }
}

/**
 * for given paramName, find *all* programs of given type that contain 
 * a parameter that matches paramName and add to paramsList
 */
void GlobalShaderParamPlugin::cacheParams(int8_t shaderType , const std::string& paramName, GpuProgramParamsList& paramsList)
{
  ResourceManager::ResourceMapIterator resourceIterator =
      Ogre::MaterialManager::getSingleton().getResourceIterator();
  while( resourceIterator.hasMoreElements() ) {
    Ogre::ResourcePtr resource = resourceIterator.getNext();
    std::string matName = resource->getName();
#if OGRE_GREATER_THAN_1_8
    Ogre::MaterialPtr material = resource.dynamicCast<Ogre::Material>(); // ogre 1.9
#else
    Ogre::MaterialPtr material = resource;
#endif
    if(!material.isNull()) {
      for(unsigned t = 0; t < material->getNumTechniques(); t++) {
        Ogre::Technique* technique = material->getTechnique(t);
        if(technique) {
          for (unsigned p = 0; p < technique->getNumPasses(); p++) {
            Ogre::Pass *pass = technique->getPass(p);
            if(pass && pass->isProgrammable()) {
              switch(shaderType) {
                case ShaderParamUpdate::SHADER_TYPE_VERTEX:
                  if(pass->hasVertexProgram()) {
                    Ogre::GpuProgramParametersSharedPtr gpuParams = pass->getVertexProgramParameters();
                    if(!gpuParams.isNull()) {
                      // if lookup of paramName succeeds, add gpuParams to list
                      const GpuConstantDefinition* paramDef = gpuParams->_findNamedConstantDefinition(paramName);
                      if(paramDef) {
                        paramsList.push_back(gpuParams);
                        // Useful for debugging, but creates too much output
                        //gzmsg << "-- " << paramName.c_str() << " found in "
                        //      << shaderTypeName(shaderType) << " program: "
                        //      << matName.c_str() << " (" << gpuParams.get() << ")\n";
                      }
                    }
                  }
                  break;
                case ShaderParamUpdate::SHADER_TYPE_FRAGMENT:
                  if(pass->hasFragmentProgram()) {
                    Ogre::GpuProgramParametersSharedPtr gpuParams = pass->getFragmentProgramParameters();
                    if(!gpuParams.isNull()) {
                      // if lookup of paramName succeeds, add gpuParams to list
                      const GpuConstantDefinition* paramDef = gpuParams->_findNamedConstantDefinition(paramName);
                      if(paramDef) {
                        paramsList.push_back(gpuParams);
                        // Useful for debugging, but creates too much output
                        //gzmsg << "-- " << paramName.c_str() << " found in "
                        //      << shaderTypeName(shaderType) << " program: "
                        //      << matName.c_str() << " (" << gpuParams.get() << ")\n";
                      }
                    }
                  }
                  break;
              }
            }
          }
        }
      }
    }
  }
  gzmsg << "GlobalShaderParamPlugin::cacheParams(" << paramName.c_str() << ", " << paramsList.size() << ")\n";
}


/**
 * 
 */
void GlobalShaderParamPlugin::onPostRender()
{
  if(m_hasUpdates) {
    Lock guard(m_mutex);
    if(m_cacheCleared) {
      buildCache();
      m_cacheCleared = false;
    }
    
    for(int8_t shaderType = 0; shaderType < ShaderParamUpdate::NUM_SHADER_TYPES; shaderType++) {
      for(auto& pair : m_paramUpdateMap[shaderType]) {
        const std::string& name = pair.first;
        const std::string& value = pair.second;
        for(auto& params : m_paramsListMap[shaderType][name]) {
          setParam(params, name, value);
        }
      }
      m_paramUpdateMap[shaderType].clear();
    }
        
    m_hasUpdates = false;
  }
}

/**
 * 
 */
void GlobalShaderParamPlugin::setParam(Ogre::GpuProgramParametersSharedPtr params, 
                                       const std::string& paramName, 
                                       const std::string& paramValue) 
{
  const GpuConstantDefinition* paramDef = params->_findNamedConstantDefinition(paramName);
  if(paramDef) {
    switch (paramDef->constType) {
      case Ogre::GCT_INT1:
      {
        int value = Ogre::StringConverter::parseInt(paramValue);
        params->setNamedConstant(paramName, value);
        break;
      }
      case Ogre::GCT_FLOAT1:
      {
        Ogre::Real value = Ogre::StringConverter::parseReal(paramValue);
        params->setNamedConstant(paramName, value);
        break;
      }
#if OGRE_GREATER_THAN_1_8
      case Ogre::GCT_INT2:
      case Ogre::GCT_FLOAT2:
      {
        Ogre::Vector2 value = Ogre::StringConverter::parseVector2(paramValue);
        params->setNamedConstant(paramName, value);
        break;
      }
#endif
      case Ogre::GCT_INT3:
      case Ogre::GCT_FLOAT3:
      {
        Ogre::Vector3 value = Ogre::StringConverter::parseVector3(paramValue);
        params->setNamedConstant(paramName, value);
        break;
      }
      case Ogre::GCT_INT4:
      case Ogre::GCT_FLOAT4:
      {
        Ogre::Vector4 value = Ogre::StringConverter::parseVector4(paramValue);
        params->setNamedConstant(paramName, value);
        break;
      }
      case Ogre::GCT_MATRIX_4X4:
      {
        Ogre::Matrix4 value = Ogre::StringConverter::parseMatrix4(paramValue);
        params->setNamedConstant(paramName, value);
        break;
      }
      default:
        break;
    }
  }
}

std::string GlobalShaderParamPlugin::shaderTypeName(int8_t type)
{
  switch(type){
  case ShaderParamUpdate::SHADER_TYPE_VERTEX:
    return "vertex";
  case ShaderParamUpdate::SHADER_TYPE_FRAGMENT:
    return "fragment";
  default:
    return "undefined";
  }
}
