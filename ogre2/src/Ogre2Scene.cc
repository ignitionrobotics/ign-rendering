/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <ignition/common/Console.hh>

#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2ArrowVisual.hh"
#include "ignition/rendering/ogre2/Ogre2AxisVisual.hh"
#include "ignition/rendering/ogre2/Ogre2Camera.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2DepthCamera.hh"
#include "ignition/rendering/ogre2/Ogre2GizmoVisual.hh"
#include "ignition/rendering/ogre2/Ogre2GpuRays.hh"
#include "ignition/rendering/ogre2/Ogre2Grid.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2Light.hh"
#include "ignition/rendering/ogre2/Ogre2LightVisual.hh"
#include "ignition/rendering/ogre2/Ogre2LidarVisual.hh"
#include "ignition/rendering/ogre2/Ogre2Marker.hh"
#include "ignition/rendering/ogre2/Ogre2Material.hh"
#include "ignition/rendering/ogre2/Ogre2MeshFactory.hh"
#include "ignition/rendering/ogre2/Ogre2Node.hh"
#include "ignition/rendering/ogre2/Ogre2ParticleEmitter.hh"
#include "ignition/rendering/ogre2/Ogre2RayQuery.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2ThermalCamera.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"
#include "ignition/rendering/ogre2/Ogre2WireBox.hh"

/// \brief Private data for the Ogre2Scene class
class ignition::rendering::Ogre2ScenePrivate
{
  /// \brief Flag to indicate if shadows need to be updated
  public: bool shadowsDirty = true;

  /// \brief Flag to indicate if sky is enabled or not
  public: bool skyEnabled = false;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2Scene::Ogre2Scene(unsigned int _id, const std::string &_name) :
  BaseScene(_id, _name), dataPtr(std::make_unique<Ogre2ScenePrivate>())
{
}

//////////////////////////////////////////////////
Ogre2Scene::~Ogre2Scene()
{
}

//////////////////////////////////////////////////
void Ogre2Scene::Fini()
{
}

//////////////////////////////////////////////////
RenderEngine *Ogre2Scene::Engine() const
{
  return Ogre2RenderEngine::Instance();
}

//////////////////////////////////////////////////
VisualPtr Ogre2Scene::RootVisual() const
{
  return this->rootVisual;
}

//////////////////////////////////////////////////
math::Color Ogre2Scene::AmbientLight() const
{
  // This method considers that the ambient upper hemisphere and
  // the lower hemisphere light configurations are the same. For
  // more info check Ogre2Scene::SetAmbientLight documentation.
  Ogre::ColourValue ogreColor =
    this->ogreSceneManager->getAmbientLightUpperHemisphere();
  return Ogre2Conversions::Convert(ogreColor);
}

//////////////////////////////////////////////////
void Ogre2Scene::SetAmbientLight(const math::Color &_color)
{
  // We set the same ambient light for both hemispheres for a
  // traditional fixed-colour ambient light.
  // https://ogrecave.github.io/ogre/api/2.1/class_ogre_1_1_scene
  // _manager.html#a56cd9aa2c4dee4eec9eb07ce1372fb52
  // It's preferred to set the hemisphereDir arg to the up axis,
  // which in our case would be Ogre::Vector3::UNIT_Z
  Ogre::ColourValue ogreColor = Ogre2Conversions::Convert(_color);
  this->ogreSceneManager->setAmbientLight(ogreColor, ogreColor,
      Ogre::Vector3::UNIT_Z);
}


//////////////////////////////////////////////////
void Ogre2Scene::PreRender()
{
  if (this->ShadowsDirty())
  {
    // notify all render targets
    for (unsigned int i  = 0; i < this->SensorCount(); ++i)
    {
      auto camera = std::dynamic_pointer_cast<Ogre2Camera>(
          this->SensorByIndex(i));
      if (camera)
      {
        Ogre2RenderTexturePtr rt =
            std::dynamic_pointer_cast<Ogre2RenderTexture>(
            camera->RenderTarget());
        if (rt)
        {
          // need to destroy compositor resources first before the render target
          // builds the new shadow node
          rt->DestroyCompositor();
          // hack: this marks the render target dirty and causes it to be
          // rebuilt
          rt->SetCamera(rt->Camera());
        }
      }
    }
  }

  BaseScene::PreRender();
}

//////////////////////////////////////////////////
void Ogre2Scene::Clear()
{
  this->meshFactory->Clear();

  BaseScene::Clear();
}

//////////////////////////////////////////////////
void Ogre2Scene::Destroy()
{
  this->DestroyNodes();

  // cleanup any items that were not attached to nodes
  // make sure to do this before destroying materials done by BaseScene::Destroy
  // otherwise ogre throws an exception when unlinking a renderable from a
  // hlms datablock
  this->ogreSceneManager->destroyAllItems();

  BaseScene::Destroy();

  if (this->ogreSceneManager)
  {
    this->ogreSceneManager->removeRenderQueueListener(
        Ogre2RenderEngine::Instance()->OverlaySystem());
  }
}

//////////////////////////////////////////////////
Ogre::SceneManager *Ogre2Scene::OgreSceneManager() const
{
  return this->ogreSceneManager;
}

//////////////////////////////////////////////////
bool Ogre2Scene::LoadImpl()
{
  return true;
}

//////////////////////////////////////////////////
bool Ogre2Scene::InitImpl()
{
  this->CreateContext();
  this->CreateRootVisual();
  this->CreateStores();
  this->CreateMeshFactory();

  return true;
}

//////////////////////////////////////////////////
LightStorePtr Ogre2Scene::Lights() const
{
  return this->lights;
}

//////////////////////////////////////////////////
SensorStorePtr Ogre2Scene::Sensors() const
{
  return this->sensors;
}

//////////////////////////////////////////////////
VisualStorePtr Ogre2Scene::Visuals() const
{
  return this->visuals;
}

//////////////////////////////////////////////////
MaterialMapPtr Ogre2Scene::Materials() const
{
  return this->materials;
}

//////////////////////////////////////////////////
DirectionalLightPtr Ogre2Scene::CreateDirectionalLightImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2DirectionalLightPtr light(new Ogre2DirectionalLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
PointLightPtr Ogre2Scene::CreatePointLightImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2PointLightPtr light(new Ogre2PointLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
SpotLightPtr Ogre2Scene::CreateSpotLightImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2SpotLightPtr light(new Ogre2SpotLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
CameraPtr Ogre2Scene::CreateCameraImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2CameraPtr camera(new Ogre2Camera);
  bool result = this->InitObject(camera, _id, _name);
  camera->SetBackgroundColor(this->backgroundColor);
  if (this->backgroundMaterial)
    camera->SetBackgroundMaterial(this->backgroundMaterial);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
DepthCameraPtr Ogre2Scene::CreateDepthCameraImpl(const unsigned int _id,
    const std::string &_name)
{
  Ogre2DepthCameraPtr camera(new Ogre2DepthCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
ThermalCameraPtr Ogre2Scene::CreateThermalCameraImpl(const unsigned int _id,
    const std::string &_name)
{
  Ogre2ThermalCameraPtr camera(new Ogre2ThermalCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
GpuRaysPtr Ogre2Scene::CreateGpuRaysImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2GpuRaysPtr gpuRays(new Ogre2GpuRays);
  bool result = this->InitObject(gpuRays, _id, _name);
  return (result) ? gpuRays : nullptr;
}

//////////////////////////////////////////////////
VisualPtr Ogre2Scene::CreateVisualImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2VisualPtr visual(new Ogre2Visual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
ArrowVisualPtr Ogre2Scene::CreateArrowVisualImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2ArrowVisualPtr visual(new Ogre2ArrowVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
AxisVisualPtr Ogre2Scene::CreateAxisVisualImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2AxisVisualPtr visual(new Ogre2AxisVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
LightVisualPtr Ogre2Scene::CreateLightVisualImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2LightVisualPtr visual(new Ogre2LightVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
GizmoVisualPtr Ogre2Scene::CreateGizmoVisualImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2GizmoVisualPtr visual(new Ogre2GizmoVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
GeometryPtr Ogre2Scene::CreateBoxImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_box");
}

//////////////////////////////////////////////////
GeometryPtr Ogre2Scene::CreateConeImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_cone");
}

//////////////////////////////////////////////////
GeometryPtr Ogre2Scene::CreateCylinderImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_cylinder");
}

//////////////////////////////////////////////////
GeometryPtr Ogre2Scene::CreatePlaneImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_plane");
}

//////////////////////////////////////////////////
GeometryPtr Ogre2Scene::CreateSphereImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_sphere");
}

//////////////////////////////////////////////////
MeshPtr Ogre2Scene::CreateMeshImpl(unsigned int _id, const std::string &_name,
    const std::string &_meshName)
{
  MeshDescriptor descriptor(_meshName);
  return this->CreateMeshImpl(_id, _name, descriptor);
}

//////////////////////////////////////////////////
MeshPtr Ogre2Scene::CreateMeshImpl(unsigned int _id,
    const std::string &_name, const MeshDescriptor &_desc)
{
  Ogre2MeshPtr mesh = this->meshFactory->Create(_desc);
  if (nullptr == mesh)
    return nullptr;

  bool result = this->InitObject(mesh, _id, _name);
  return (result) ? mesh : nullptr;
}

//////////////////////////////////////////////////
HeightmapPtr Ogre2Scene::CreateHeightmapImpl(unsigned int,
    const std::string &, const HeightmapDescriptor &)
{
  ignerr << "Ogre 2 doesn't support heightmaps yet, see " <<
      "https://github.com/ignitionrobotics/ign-rendering/issues/187"
      << std::endl;
  return nullptr;
}

//////////////////////////////////////////////////
GridPtr Ogre2Scene::CreateGridImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2GridPtr grid(new Ogre2Grid);
  bool result = this->InitObject(grid, _id, _name);
  return (result) ? grid : nullptr;
}

//////////////////////////////////////////////////
WireBoxPtr Ogre2Scene::CreateWireBoxImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2WireBoxPtr wireBox(new Ogre2WireBox);
  bool result = this->InitObject(wireBox, _id, _name);
  return (result) ? wireBox: nullptr;
}

//////////////////////////////////////////////////
MarkerPtr Ogre2Scene::CreateMarkerImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2MarkerPtr marker(new Ogre2Marker);
  bool result = this->InitObject(marker, _id, _name);
  return (result) ? marker: nullptr;
}

//////////////////////////////////////////////////
LidarVisualPtr Ogre2Scene::CreateLidarVisualImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2LidarVisualPtr lidar(new Ogre2LidarVisual);
  bool result = this->InitObject(lidar, _id, _name);
  return (result) ? lidar: nullptr;
}

//////////////////////////////////////////////////
TextPtr Ogre2Scene::CreateTextImpl(unsigned int /*_id*/,
    const std::string &/*_name*/)
{
  // TODO(anyone)
  return TextPtr();
}

//////////////////////////////////////////////////
MaterialPtr Ogre2Scene::CreateMaterialImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2MaterialPtr material(new Ogre2Material);
  bool result = this->InitObject(material, _id, _name);
  return (result) ? material : nullptr;
}

//////////////////////////////////////////////////
RenderTexturePtr Ogre2Scene::CreateRenderTextureImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2RenderTexturePtr renderTexture(new Ogre2RenderTexture);
  bool result = this->InitObject(renderTexture, _id, _name);
  return (result) ? renderTexture : nullptr;
}

//////////////////////////////////////////////////
RenderWindowPtr Ogre2Scene::CreateRenderWindowImpl(unsigned int /*_id*/,
    const std::string &/*_name*/)
{
  // TODO(anyone)
  return RenderWindowPtr();
}

//////////////////////////////////////////////////
RayQueryPtr Ogre2Scene::CreateRayQueryImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2RayQueryPtr rayQuery(new Ogre2RayQuery);
  bool result = this->InitObject(rayQuery, _id, _name);
  return (result) ? rayQuery : nullptr;
}

//////////////////////////////////////////////////
ParticleEmitterPtr Ogre2Scene::CreateParticleEmitterImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2ParticleEmitterPtr visual(new Ogre2ParticleEmitter);
  bool result = this->InitObject(visual, _id, _name);

  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
bool Ogre2Scene::InitObject(Ogre2ObjectPtr _object, unsigned int _id,
    const std::string &_name)
{
  // assign needed varibles
  _object->id = _id;
  _object->name = _name;
  _object->scene = this->SharedThis();

  // initialize object
  _object->Load();
  _object->Init();

  return true;
}

//////////////////////////////////////////////////
void Ogre2Scene::CreateContext()
{
  std::cerr << "/* CreateContext */" << '\n';
  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();

  // getNumLogicalCores() may return 0 if couldn't detect
  const size_t numThreads = std::max<size_t>(
      1, Ogre::PlatformInformation::getNumLogicalCores());

  // See ogre doxygen documentation regarding culling methods.
  // In some cases you may still want to use single thread.
  // if( numThreads > 1 )
  //   threadedCullingMethod = Ogre::INSTANCING_CULLING_THREADED;
  // Create the SceneManager, in this case a generic one
  this->ogreSceneManager = root->createSceneManager(Ogre::ST_GENERIC,
                                                    numThreads);

  this->ogreSceneManager->addRenderQueueListener(
      Ogre2RenderEngine::Instance()->OverlaySystem());

  this->ogreSceneManager->getRenderQueue()->setSortRenderQueue(
      Ogre::v1::OverlayManager::getSingleton().mDefaultRenderQueueId,
      Ogre::RenderQueue::StableSort);

  // Set sane defaults for proper shadow mapping
  this->ogreSceneManager->setShadowDirectionalLightExtrusionDistance(500.0f);
  this->ogreSceneManager->setShadowFarDistance(500.0f);

  // enable forward plus to support multiple lights
  // this is required for non-shadow-casting point lights and
  // spot lights to work
  this->ogreSceneManager->setForwardClustered(true, 16, 8, 24, 96, 0, 0, 1, 500);
}

//////////////////////////////////////////////////
void Ogre2Scene::CreateRootVisual()
{
  if (this->rootVisual)
    return;

  // create unregistered visual
  this->rootVisual = Ogre2VisualPtr(new Ogre2Visual);
  unsigned int rootId = this->CreateObjectId();
  std::string rootName = this->CreateObjectName(rootId, "_ROOT_");

  // check if root visual created successfully
  if (!this->InitObject(this->rootVisual, rootId, rootName))
  {
    ignerr << "Unable to create root visual" << std::endl;
    this->rootVisual = nullptr;
    return;
  }

  // add visual node to actual ogre root
  Ogre::SceneNode *ogreRootNode = this->rootVisual->Node();
  this->ogreSceneManager->getRootSceneNode()->addChild(ogreRootNode);
}

//////////////////////////////////////////////////
void Ogre2Scene::CreateMeshFactory()
{
  Ogre2ScenePtr sharedThis = this->SharedThis();
  this->meshFactory = Ogre2MeshFactoryPtr(new Ogre2MeshFactory(sharedThis));
}

//////////////////////////////////////////////////
void Ogre2Scene::CreateStores()
{
  this->lights = Ogre2LightStorePtr(new Ogre2LightStore);
  this->sensors = Ogre2SensorStorePtr(new Ogre2SensorStore);
  this->visuals = Ogre2VisualStorePtr(new Ogre2VisualStore);
  this->materials = Ogre2MaterialMapPtr(new Ogre2MaterialMap);
}

//////////////////////////////////////////////////
Ogre2ScenePtr Ogre2Scene::SharedThis()
{
  ScenePtr sharedBase = this->shared_from_this();
  return std::dynamic_pointer_cast<Ogre2Scene>(sharedBase);
}

//////////////////////////////////////////////////
void Ogre2Scene::SetShadowsDirty(bool _dirty)
{
  this->dataPtr->shadowsDirty = _dirty;
}

//////////////////////////////////////////////////
bool Ogre2Scene::ShadowsDirty() const
{
  return this->dataPtr->shadowsDirty;
}

//////////////////////////////////////////////////
void Ogre2Scene::SetSkyEnabled(bool _enabled)
{
  MaterialPtr skyboxMat;
  if (_enabled)
  {
    // get skybox material
    std::string skyboxMatName = "Default/skybox";
    skyboxMat = this->Material(skyboxMatName);
    if (!skyboxMat)
    {
      // ogre2 should be able to find this texture as resource search
      // paths are already set up in Ogre2RenderEngine.cc
      std::string skyboxEnvMap = "skybox.dds";
      skyboxMat = this->CreateMaterial(skyboxMatName);
      skyboxMat->SetEnvironmentMap(skyboxEnvMap);
    }
  }
  this->SetBackgroundMaterial(skyboxMat);
  for (unsigned int i = 0; i < this->Sensors()->Size(); ++i)
  {
    auto sensor = this->Sensors()->GetByIndex(i);
    auto camera = std::dynamic_pointer_cast<Ogre2Camera>(sensor);
    if (camera)
    {
      camera->SetBackgroundMaterial(skyboxMat);
    }
  }
  this->dataPtr->skyEnabled = _enabled;
}

//////////////////////////////////////////////////
bool Ogre2Scene::SkyEnabled() const
{
  return this->dataPtr->skyEnabled;
}
