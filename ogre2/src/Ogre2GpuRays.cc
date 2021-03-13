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

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/common/Console.hh>
#include <ignition/math/Helpers.hh>

#include "ignition/rendering/ogre2/Ogre2Camera.hh"
#include "ignition/rendering/ogre2/Ogre2GpuRays.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2ParticleEmitter.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Sensor.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"

#include "Ogre2ParticleNoiseListener.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
//
/// \brief Helper class for switching the ogre item's material to laser retro
/// source material when a thermal camera is being rendered.
class Ogre2LaserRetroMaterialSwitcher : public Ogre::RenderTargetListener
{
  /// \brief constructor
  /// \param[in] _scene the scene manager responsible for rendering
  public: explicit Ogre2LaserRetroMaterialSwitcher(Ogre2ScenePtr _scene);

  /// \brief destructor
  public: ~Ogre2LaserRetroMaterialSwitcher() = default;

  /// \brief Callback when a render target is about to be rendered
  /// \param[in] _evt Ogre render target event containing information about
  /// the source render target.
  private: virtual void preRenderTargetUpdate(
      const Ogre::RenderTargetEvent &_evt) override;

  /// \brief Callback when a render target is finisned being rendered
  /// \param[in] _evt Ogre render target event containing information about
  /// the source render target.
  private: virtual void postRenderTargetUpdate(
      const Ogre::RenderTargetEvent &_evt) override;

  /// \brief Scene manager
  private: Ogre2ScenePtr scene = nullptr;

  /// \brief Pointer to the laser retro source material
  private: Ogre::MaterialPtr laserRetroSourceMaterial;

  /// \brief Custom parameter index of laser retro value in an ogre subitem.
  /// This has to match the custom index specifed in LaserRetroSource material
  /// script in media/materials/scripts/gpu_rays.material
  private: const unsigned int customParamIdx = 10u;

  /// \brief A map of ogre sub item pointer to their original hlms material
  private: std::map<Ogre::SubItem *, Ogre::HlmsDatablock *> datablockMap;
};
}
}
}


/// \internal
/// \brief Private data for the Ogre2GpuRays class
class ignition::rendering::Ogre2GpuRaysPrivate
{
  /// \brief Event triggered when new gpu rays range data are available.
  /// \param[in] _frame New frame containing raw gpu rays data.
  /// \param[in] _width Width of frame.
  /// \param[in] _height Height of frame.
  /// \param[in] _channel Number of channels
  /// \param[in] _format Format of frame.
  public: ignition::common::EventT<void(const float *,
               unsigned int, unsigned int, unsigned int,
               const std::string &)> newGpuRaysFrame;

  /// \brief Raw buffer of gpu rays data.
  public: float *gpuRaysBuffer = nullptr;

  /// \brief Outgoing gpu rays data, used by newGpuRaysFrame event.
  public: float *gpuRaysScan = nullptr;

  /// \brief Pointer to Ogre material for the first rendering pass.
  public: Ogre::MaterialPtr matFirstPass;

  /// \brief Pointer to Ogre material for the second rendering pass.
  public: Ogre::MaterialPtr matSecondPass;

  /// \brief Cubemap cameras
  public: Ogre::Camera *cubeCam[6];

  /// \brief Texture packed with cubemap face and uv data
  public: Ogre::TexturePtr cubeUVTexture;

  /// \brief Set of cubemap faces that are needed to generate the final
  /// range data
  public: std::set<unsigned int> cubeFaceIdx;

  /// \brief 1st pass compositor workspace definition
  public: std::string ogreCompositorWorkspaceDef1st;

  /// \brief 1st pass compositor node definition
  public: std::string ogreCompositorNodeDef1st;

  /// \brief 1st pass compositor workspace. One for each cubemap camera
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace1st[6];

  /// \brief 2nd pass compositor workspace definition
  public: std::string ogreCompositorWorkspaceDef2nd;

  /// \brief 1st pass compositor node definition
  public: std::string ogreCompositorNodeDef2nd;

  /// \brief 2nd pass compositor workspace.
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace2nd = nullptr;

  /// \brief An array of first pass textures. One for each cubemap camera.
  public: Ogre::TexturePtr firstPassTextures[6];

  /// \brief Second pass texture.
  public: Ogre::TexturePtr secondPassTexture;

  /// \brief Pointer to the ogre camera
  public: Ogre::Camera *ogreCamera = nullptr;

  /// \brief Image width of first pass.
  public: unsigned int w1st = 0u;

  /// \brief Image height of first pass.
  public: unsigned int h1st = 0u;

  /// \brief Image width of second pass.
  public: unsigned int w2nd = 0u;

  /// \brief Image height of second pass.
  public: unsigned int h2nd = 0u;

  /// \brief Dummy render texture for the gpu rays
  public: RenderTexturePtr renderTexture;

  /// \brief Pointer to material switcher
  public: std::unique_ptr<Ogre2LaserRetroMaterialSwitcher>
      laserRetroMaterialSwitcher[6];

  /// \brief standard deviation of particle noise
  public: double particleStddev = 0.01;

  /// \brief Listener for setting particle noise value based on particle
  /// emitter region
  public: std::unique_ptr<Ogre2ParticleNoiseListener> particleNoiseListener[6];
};

using namespace ignition;
using namespace rendering;


//////////////////////////////////////////////////
Ogre2LaserRetroMaterialSwitcher::Ogre2LaserRetroMaterialSwitcher(
    Ogre2ScenePtr _scene)
{
  this->scene = _scene;
  // plain opaque material
  Ogre::ResourcePtr res =
    Ogre::MaterialManager::getSingleton().load("LaserRetroSource",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  this->laserRetroSourceMaterial = res.staticCast<Ogre::Material>();
  this->laserRetroSourceMaterial->load();
}

//////////////////////////////////////////////////
void Ogre2LaserRetroMaterialSwitcher::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent & /*_evt*/)
{
  // swap item to use v1 shader material
  // Note: keep an eye out for performance impact on switching materials
  // on the fly. We are not doing this often so should be ok.
  this->datablockMap.clear();
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);
  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);

    std::string laserRetroKey = "laser_retro";
    // get visual
    Ogre::Any userAny = item->getUserObjectBindings().getUserAny();
    if (!userAny.isEmpty() && userAny.getType() == typeid(unsigned int))
    {
      VisualPtr result;
      try
      {
        result = this->scene->VisualById(Ogre::any_cast<unsigned int>(userAny));
      }
      catch(Ogre::Exception &e)
      {
        ignerr << "Ogre Error:" << e.getFullDescription() << "\n";
      }
      Ogre2VisualPtr ogreVisual =
          std::dynamic_pointer_cast<Ogre2Visual>(result);

      // get laser_retro
      Variant tempLaserRetro = ogreVisual->UserData(laserRetroKey);

        float retroValue = -1.0;
        try
        {
          retroValue = std::get<float>(tempLaserRetro);
        }
        catch(...)
        {
          try
          {
            retroValue = std::get<double>(tempLaserRetro);
          }
          catch(...)
          {
            try
            {
              retroValue = std::get<int>(tempLaserRetro);
            }
            catch(std::bad_variant_access &e)
            {
              ignerr << "Error casting user data: " << e.what() << "\n";
              retroValue = -1.0;
            }
          }
        }

        // only accept positive laser retro value
        if (retroValue >= 0)
        {
          // set visibility flag so the camera can see it
          item->addVisibilityFlags(0x01000000);
          for (unsigned int i = 0; i < item->getNumSubItems(); ++i)
          {
            Ogre::SubItem *subItem = item->getSubItem(i);
            if (!subItem->hasCustomParameter(this->customParamIdx))
            {
              // limit laser retro value to 2000 (as in gazebo)
              if (retroValue > 2000.0)
              {
                retroValue = 2000.0;
              }
              float color = retroValue / 2000.0;
              subItem->setCustomParameter(this->customParamIdx,
                  Ogre::Vector4(color, color, color, 1.0));
            }
            Ogre::HlmsDatablock *datablock = subItem->getDatablock();
            this->datablockMap[subItem] = datablock;

            subItem->setMaterial(this->laserRetroSourceMaterial);
          }
        }
      }
      itor.moveNext();
    }
  }

//////////////////////////////////////////////////
void Ogre2LaserRetroMaterialSwitcher::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent & /*_evt*/)
{
  // restore item to use hlms material
  for (auto it : this->datablockMap)
  {
    Ogre::SubItem *subItem = it.first;
    subItem->setDatablock(it.second);
  }
}


//////////////////////////////////////////////////
Ogre2GpuRays::Ogre2GpuRays()
  : dataPtr(new Ogre2GpuRaysPrivate)
{
  // r = depth, g = retro, and b = n/a
  this->channels = 3u;

  for (unsigned int i = 0; i < 6u; ++i)
  {
    this->dataPtr->cubeCam[i] = nullptr;
    this->dataPtr->ogreCompositorWorkspace1st[i] = nullptr;
    this->dataPtr->laserRetroMaterialSwitcher[i] = nullptr;
  }
}

//////////////////////////////////////////////////
Ogre2GpuRays::~Ogre2GpuRays()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2GpuRays::Init()
{
  BaseGpuRays::Init();

  // create internal camera
  this->CreateCamera();

  // create dummy render texture
  this->CreateRenderTexture();
}

//////////////////////////////////////////////////
void Ogre2GpuRays::Destroy()
{
  if (this->dataPtr->gpuRaysBuffer)
  {
    delete [] this->dataPtr->gpuRaysBuffer;
    this->dataPtr->gpuRaysBuffer = nullptr;
  }

  if (this->dataPtr->gpuRaysScan)
  {
    delete [] this->dataPtr->gpuRaysScan;
    this->dataPtr->gpuRaysScan = nullptr;
  }

  if (this->dataPtr->cubeUVTexture)
  {
    Ogre::TextureManager::getSingleton().remove(
        this->dataPtr->cubeUVTexture->getName());
    this->dataPtr->cubeUVTexture.reset();
  }

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  // remove 1st pass textures, material, compositors
  for (auto i : this->dataPtr->cubeFaceIdx)
  {
    if (this->dataPtr->firstPassTextures[i])
    {
      Ogre::TextureManager::getSingleton().remove(
          this->dataPtr->firstPassTextures[i]->getName());
      this->dataPtr->firstPassTextures[i].reset();
    }
    if (this->dataPtr->ogreCompositorWorkspace1st[i])
    {
      ogreCompMgr->removeWorkspace(
          this->dataPtr->ogreCompositorWorkspace1st[i]);
      this->dataPtr->ogreCompositorWorkspace1st[i] = nullptr;
    }
  }
  if (this->dataPtr->matFirstPass)
  {
    Ogre::MaterialManager::getSingleton().remove(
        this->dataPtr->matFirstPass->getName());
    this->dataPtr->matFirstPass.reset();
  }

  if (!this->dataPtr->ogreCompositorWorkspaceDef1st.empty())
  {
    ogreCompMgr->removeWorkspaceDefinition(
        this->dataPtr->ogreCompositorWorkspaceDef1st);
    ogreCompMgr->removeNodeDefinition(
        this->dataPtr->ogreCompositorNodeDef1st);
    this->dataPtr->ogreCompositorWorkspaceDef1st.clear();
  }

  // remove 2nd pass texture, material, compositor
  if (this->dataPtr->secondPassTexture)
  {
    Ogre::TextureManager::getSingleton().remove(
        this->dataPtr->secondPassTexture->getName());
    this->dataPtr->secondPassTexture.reset();
  }

  if (this->dataPtr->matSecondPass)
  {
    Ogre::MaterialManager::getSingleton().remove(
        this->dataPtr->matSecondPass->getName());
    this->dataPtr->matSecondPass.reset();
  }

  if (!this->dataPtr->ogreCompositorWorkspaceDef2nd.empty())
  {
    ogreCompMgr->removeWorkspace(this->dataPtr->ogreCompositorWorkspace2nd);
    ogreCompMgr->removeWorkspaceDefinition(
        this->dataPtr->ogreCompositorWorkspaceDef2nd);
    ogreCompMgr->removeNodeDefinition(
        this->dataPtr->ogreCompositorNodeDef2nd);
    this->dataPtr->ogreCompositorWorkspaceDef2nd.clear();
  }
}

/////////////////////////////////////////////////
void Ogre2GpuRays::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->dataPtr->renderTexture =
      std::dynamic_pointer_cast<Ogre2RenderTexture>(base);
  this->dataPtr->renderTexture->SetWidth(1);
  this->dataPtr->renderTexture->SetHeight(1);
}

/////////////////////////////////////////////////
void Ogre2GpuRays::CreateCamera()
{
  // Create ogre camera object
  Ogre::SceneManager *ogreSceneManager  = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->dataPtr->ogreCamera = ogreSceneManager->createCamera(
      this->Name() + "_Camera");
  if (this->dataPtr->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  // by default, ogre2 cameras are attached to root scene node
  this->dataPtr->ogreCamera->detachFromParent();
  this->ogreNode->attachObject(this->dataPtr->ogreCamera);
  this->dataPtr->ogreCamera->setFixedYawAxis(false);
  this->dataPtr->ogreCamera->yaw(Ogre::Degree(-90));
  this->dataPtr->ogreCamera->roll(Ogre::Degree(-90));
  this->dataPtr->ogreCamera->setAutoAspectRatio(true);
}

/////////////////////////////////////////////////
void Ogre2GpuRays::ConfigureCamera()
{
  // horizontal gpu rays setup
  this->SetHFOV(this->AngleMax() - this->AngleMin());

  // vertical laser setup
  double vfovAngle;

  if (this->VerticalRangeCount() > 1)
  {
    vfovAngle = (this->VerticalAngleMax() - this->VerticalAngleMin()).Radian();
  }
  else
  {
    vfovAngle = 0;

    if (this->VerticalAngleMax() != this->VerticalAngleMin())
    {
      ignwarn << "Only one vertical ray but vertical min. and max. angle "
          "are not equal. Min. angle is used.\n";
      this->SetVerticalAngleMax(this->VerticalAngleMin().Radian());
    }
  }
  this->SetVFOV(vfovAngle);

  // Configure first pass texture size
  this->Set1stTextureSize(1024, 1024);

  // Configure second pass texture size
  this->SetRangeCount(this->RangeCount(), this->VerticalRangeCount());

  // Set ogre cam properties
  this->dataPtr->ogreCamera->setNearClipDistance(this->NearClipPlane());
  this->dataPtr->ogreCamera->setFarClipDistance(this->FarClipPlane());
}

/////////////////////////////////////////////////////////
math::Vector2d Ogre2GpuRays::SampleCubemap(const math::Vector3d &_v,
    unsigned int &_faceIndex)
{
  math::Vector3d vAbs = _v.Abs();
  double ma;
  math::Vector2d uv;
  if (vAbs.Z() >= vAbs.X() && vAbs.Z() >= vAbs.Y())
  {
    _faceIndex = _v.Z() < 0.0 ? 5.0 : 4.0;
    ma = 0.5 / vAbs.Z();
    uv = math::Vector2d(_v.Z() < 0.0 ? -_v.X() : _v.X(), -_v.Y());
  }
  else if (vAbs.Y() >= vAbs.X())
  {
    _faceIndex = _v.Y() < 0.0 ? 3.0 : 2.0;
    ma = 0.5 / vAbs.Y();
    uv = math::Vector2d(_v.X(), _v.Y() < 0.0 ? -_v.Z() : _v.Z());
  }
  else
  {
    _faceIndex = _v.X() < 0.0 ? 1.0 : 0.0;
    ma = 0.5 / vAbs.X();
    uv = math::Vector2d(_v.X() < 0.0 ? _v.Z() : -_v.Z(), -_v.Y());
  }
  return uv * ma + 0.5;
}

/////////////////////////////////////////////////////////
void Ogre2GpuRays::CreateSampleTexture()
{
  double min = this->AngleMin().Radian();
  double max = this->AngleMax().Radian();
  double vmin = this->VerticalAngleMin().Radian();
  double vmax = this->VerticalAngleMax().Radian();
  double hStep = (max-min) / static_cast<double>(this->dataPtr->w2nd-1);
  double vStep = 1.0;
  // non-planar case
  if (this->dataPtr->h2nd > 1)
    vStep = (vmax-vmin) / static_cast<double>(this->dataPtr->h2nd-1);

  // create an RGB texture (cubeUVTex) to pack info that tells the shaders how
  // to sample from the cubemap textures.
  // Each pixel packs the follow data:
  //   R: u coordinate on the cubemap face
  //   G: v coordinate on the cubemap face
  //   B: cubemap face index
  // this texture is passed to the 2nd pass fragment shader
  std::string texName = this->Name() + "_samplerTex";
  this->dataPtr->cubeUVTexture =
      Ogre::TextureManager::getSingleton().createManual(
          texName,
          "General",
          Ogre::TEX_TYPE_2D,
          this->dataPtr->w2nd,
          this->dataPtr->h2nd,
          0,
          Ogre::PF_FLOAT32_RGB);
  Ogre::v1::HardwarePixelBufferSharedPtr pixelBuffer =
      this->dataPtr->cubeUVTexture->getBuffer();
  // fill the texture
  pixelBuffer->lock(Ogre::v1::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();
  float *pDest = static_cast<float *>(pixelBox.data);

  double v = vmin;
  for (unsigned int i = 0; i < this->dataPtr->h2nd; ++i)
  {
    double h = min;
    for (unsigned int j = 0; j < this->dataPtr->w2nd; ++j)
    {
      // set up dir vector to sample from a standard Y up cubemap
      math::Vector3d ray(0, 0, 1);
      ray.Normalize();
      math::Quaterniond pitch(math::Vector3d(1, 0, 0), -v);
      math::Quaterniond yaw(math::Vector3d(0, 1, 0), -h);
      math::Vector3d dir = yaw * pitch * ray;
      unsigned int faceIdx;
      math::Vector2d uv = this->SampleCubemap(dir, faceIdx);
      this->dataPtr->cubeFaceIdx.insert(faceIdx);
      // igndbg << "p(" << pitch << ") y(" << yaw << "): " << dir << " | "
      //       << uv << " | " << faceIdx << std::endl;
      // u
      *pDest++ = uv.X();
      // v
      *pDest++ = uv.Y();
      // face
      *pDest++ = faceIdx;

       h += hStep;
    }
    v += vStep;
  }

  pixelBuffer->unlock();
}

/////////////////////////////////////////////////////////
void Ogre2GpuRays::Setup1stPass()
{
  // Load 1st pass material
  // The GpuRaysScan1st material is defined in script (gpu_rays.material).
  // We need to clone it since we are going to modify its uniform variables
  std::string mat1stName = "GpuRaysScan1st";
  Ogre::MaterialPtr mat1st =
      Ogre::MaterialManager::getSingleton().getByName(mat1stName);
  this->dataPtr->matFirstPass = mat1st->clone(this->Name() + "_" + mat1stName);
  this->dataPtr->matFirstPass->load();
  Ogre::Pass *pass = this->dataPtr->matFirstPass->getTechnique(0)->getPass(0);
  Ogre::GpuProgramParametersSharedPtr psParams =
      pass->getFragmentProgramParameters();

  // Set the uniform variables (see gpu_rays_1st_pass_fs.glsl).
  // The projectParams is used to linearize depth buffer data
  // The other params are used to clamp the range output
  double projectionA = this->FarClipPlane() /
      (this->FarClipPlane() - this->NearClipPlane());
  double projectionB = (-this->FarClipPlane() * this->NearClipPlane()) /
      (this->FarClipPlane() - this->NearClipPlane());
  projectionB /= this->FarClipPlane();
  psParams->setNamedConstant("projectionParams",
      Ogre::Vector2(projectionA, projectionB));
  psParams->setNamedConstant("near",
      static_cast<float>(this->NearClipPlane()));
  psParams->setNamedConstant("far",
      static_cast<float>(this->FarClipPlane()));
  psParams->setNamedConstant("max",
      static_cast<float>(this->dataMaxVal));
  psParams->setNamedConstant("min",
      static_cast<float>(this->dataMinVal));
  psParams->setNamedConstant("particleStddev",
    static_cast<float>(this->dataPtr->particleStddev));

  // Create 1st pass compositor
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  // We need to programmatically create the compositor because we need to
  // configure it to use the cloned 1st pass material created earlier.
  // The compositor workspace definition is equivalent to the following
  // ogre compositor script:
  // compositor_node GpuRays1stPass
  // {
  //   in 0 rt_input
  //   texture depthTexture target_width target_height PF_D32_FLOAT
  //   texture colorTexture target_width target_height PF_R8G8B8
  //   texture particleTexture target_width target_height PF_L8
  //   texture particleDepthTexture target_width target_height PF_D32_FLOAT
  //   target colorTexture
  //   {
  //     pass clear
  //     {
  //       colour_value 0.0 0.0 0.0 1.0
  //     }
  //     pass render_scene
  //     {
  //       visibility_mask 0x11011111
  //     }
  //   }
  //   target particleTexture
  //   {
  //     pass clear
  //     {
  //       colour_value 0.0 0.0 0.0 1.0
  //     }
  //     pass render_scene
  //     {
  //       visibility_mask 0.00100000
  //     }
  //   }
  //   target rt_input
  //   {
  //     pass clear
  //     {
  //       colour_value 0.0 0.0 0.0 1.0
  //     }
  //     pass render_quad
  //     {
  //       material GpuRaysScan1st // Use copy instead of original
  //       input 0 depthTexture
  //       input 1 colorTexture
  //       quad_normals camera_far_corners_view_space
  //     }
  //   }
  //   out 0 rt_input
  // }
  std::string wsDefName = "GpuRays1stPassWorkspace_" + this->Name();
  this->dataPtr->ogreCompositorWorkspaceDef1st = wsDefName;
  if (!ogreCompMgr->hasWorkspaceDefinition(wsDefName))
  {
    std::string nodeDefName = wsDefName + "/Node";
    this->dataPtr->ogreCompositorNodeDef1st = nodeDefName;
    Ogre::CompositorNodeDef *nodeDef =
        ogreCompMgr->addNodeDefinition(nodeDefName);
    // Input texture
    nodeDef->addTextureSourceName("rt_input", 0,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    Ogre::TextureDefinitionBase::TextureDefinition *depthTexDef =
        nodeDef->addTextureDefinition("depthTexture");
    depthTexDef->textureType = Ogre::TEX_TYPE_2D;
    depthTexDef->width = 0;
    depthTexDef->height = 0;
    depthTexDef->depth = 1;
    depthTexDef->numMipmaps = 0;
    depthTexDef->widthFactor = 1;
    depthTexDef->heightFactor = 1;
    depthTexDef->formatList = {Ogre::PF_D32_FLOAT};
    depthTexDef->fsaa = 0;
    depthTexDef->uav = false;
    depthTexDef->automipmaps = false;
    depthTexDef->hwGammaWrite = Ogre::TextureDefinitionBase::BoolFalse;
    depthTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;

    depthTexDef->depthBufferFormat = Ogre::PF_UNKNOWN;
    depthTexDef->fsaaExplicitResolve = false;

    Ogre::TextureDefinitionBase::TextureDefinition *colorTexDef =
        nodeDef->addTextureDefinition("colorTexture");
    colorTexDef->textureType = Ogre::TEX_TYPE_2D;
    colorTexDef->width = 0;
    colorTexDef->height = 0;
    colorTexDef->depth = 1;
    colorTexDef->numMipmaps = 0;
    colorTexDef->widthFactor = 1;
    colorTexDef->heightFactor = 1;
    colorTexDef->formatList = {Ogre::PF_R8G8B8};
    colorTexDef->fsaa = 0;
    colorTexDef->uav = false;
    colorTexDef->automipmaps = false;
    colorTexDef->hwGammaWrite = Ogre::TextureDefinitionBase::BoolFalse;
    colorTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
    colorTexDef->depthBufferFormat = Ogre::PF_D32_FLOAT;
    colorTexDef->preferDepthTexture = true;
    colorTexDef->fsaaExplicitResolve = false;

    Ogre::TextureDefinitionBase::TextureDefinition *particleDepthTexDef =
        nodeDef->addTextureDefinition("particleDepthTexture");
    particleDepthTexDef->textureType = Ogre::TEX_TYPE_2D;
    particleDepthTexDef->width = 0;
    particleDepthTexDef->height = 0;
    particleDepthTexDef->depth = 1;
    particleDepthTexDef->numMipmaps = 0;
    particleDepthTexDef->widthFactor = 0.5;
    particleDepthTexDef->heightFactor = 0.5;
    particleDepthTexDef->formatList = {Ogre::PF_D32_FLOAT};
    particleDepthTexDef->fsaa = 0;
    particleDepthTexDef->uav = false;
    particleDepthTexDef->automipmaps = false;
    particleDepthTexDef->hwGammaWrite = Ogre::TextureDefinitionBase::BoolFalse;
    particleDepthTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;

    Ogre::TextureDefinitionBase::TextureDefinition *particleTexDef =
        nodeDef->addTextureDefinition("particleTexture");
    particleTexDef->textureType = Ogre::TEX_TYPE_2D;
    particleTexDef->width = 0;
    particleTexDef->height = 0;
    particleTexDef->depth = 1;
    particleTexDef->numMipmaps = 0;
    particleTexDef->widthFactor = 0.5;
    particleTexDef->heightFactor = 0.5;
    particleTexDef->formatList = {Ogre::PF_R8G8B8};
    particleTexDef->fsaa = 0;
    particleTexDef->uav = false;
    particleTexDef->automipmaps = false;
    particleTexDef->hwGammaWrite = Ogre::TextureDefinitionBase::BoolFalse;
    particleTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
    particleTexDef->depthBufferFormat = Ogre::PF_D32_FLOAT;
    particleTexDef->preferDepthTexture = true;
    particleTexDef->fsaaExplicitResolve = false;

    nodeDef->setNumTargetPass(3);

    Ogre::CompositorTargetDef *colorTargetDef =
        nodeDef->addTargetPass("colorTexture");
    colorTargetDef->setNumPasses(2);
    {
      // clear pass
      Ogre::CompositorPassClearDef *passClear =
          static_cast<Ogre::CompositorPassClearDef *>(
          colorTargetDef->addPass(Ogre::PASS_CLEAR));
      passClear->mColourValue = Ogre::ColourValue(0, 0, 0);
      // scene pass
      Ogre::CompositorPassSceneDef *passScene =
          static_cast<Ogre::CompositorPassSceneDef *>(
          colorTargetDef->addPass(Ogre::PASS_SCENE));
      // set camera custom visibility mask when rendering laser retro
      passScene->mVisibilityMask = 0x01000000 &
          ~Ogre2ParticleEmitter::kParticleVisibilityFlags;
    }

    Ogre::CompositorTargetDef *particleTargetDef =
        nodeDef->addTargetPass("particleTexture");
    particleTargetDef->setNumPasses(2);
    {
      // clear pass
      Ogre::CompositorPassClearDef *passClear =
          static_cast<Ogre::CompositorPassClearDef *>(
          particleTargetDef->addPass(Ogre::PASS_CLEAR));
      passClear->mColourValue = Ogre::ColourValue::Black;
      // scene pass
      Ogre::CompositorPassSceneDef *passScene =
          static_cast<Ogre::CompositorPassSceneDef *>(
          particleTargetDef->addPass(Ogre::PASS_SCENE));
      // set camera custom visibility mask when rendering laser retro
      passScene->mVisibilityMask =
          Ogre2ParticleEmitter::kParticleVisibilityFlags;
    }

    // rt_input target - converts depth to range
    Ogre::CompositorTargetDef *inputTargetDef =
        nodeDef->addTargetPass("rt_input");
    inputTargetDef->setNumPasses(2);
    {
      // clear pass
      Ogre::CompositorPassClearDef *passClear =
          static_cast<Ogre::CompositorPassClearDef *>(
          inputTargetDef->addPass(Ogre::PASS_CLEAR));
      passClear->mColourValue = Ogre::ColourValue(this->dataMaxVal, 0, 1.0);
      // quad pass
      Ogre::CompositorPassQuadDef *passQuad =
          static_cast<Ogre::CompositorPassQuadDef *>(
          inputTargetDef->addPass(Ogre::PASS_QUAD));
      passQuad->mMaterialName = this->dataPtr->matFirstPass->getName();
      passQuad->addQuadTextureSource(0, "depthTexture", 0);
      passQuad->addQuadTextureSource(1, "colorTexture", 0);
      passQuad->addQuadTextureSource(2, "particleDepthTexture", 0);
      passQuad->addQuadTextureSource(3, "particleTexture", 0);
      passQuad->mFrustumCorners =
          Ogre::CompositorPassQuadDef::VIEW_SPACE_CORNERS;
    }
    nodeDef->mapOutputChannel(0, "rt_input");
    Ogre::CompositorWorkspaceDef *workDef =
        ogreCompMgr->addWorkspaceDefinition(wsDefName);
    workDef->connectExternal(0, nodeDef->getName(), 0);
  }
  Ogre::CompositorWorkspaceDef *wsDef =
      ogreCompMgr->getWorkspaceDefinition(wsDefName);

  if (!wsDef)
  {
    ignerr << "Unable to add workspace definition [" << wsDefName << "] "
           << " for " << this->Name();
  }

  // create cubemap cameras and render to texture using 1st pass compositor
  Ogre::SceneManager *ogreSceneManager = this->scene->OgreSceneManager();
  for (auto i : this->dataPtr->cubeFaceIdx)
  {
    this->dataPtr->cubeCam[i] = ogreSceneManager->createCamera(
        this->Name() + "_env" + std::to_string(i));
    this->dataPtr->cubeCam[i]->detachFromParent();
    this->ogreNode->attachObject(this->dataPtr->cubeCam[i]);
    this->dataPtr->cubeCam[i]->setFOVy(Ogre::Degree(90));
    this->dataPtr->cubeCam[i]->setAspectRatio(1);
    this->dataPtr->cubeCam[i]->setNearClipDistance(this->NearClipPlane());
    this->dataPtr->cubeCam[i]->setFarClipDistance(this->FarClipPlane());
    this->dataPtr->cubeCam[i]->setFixedYawAxis(false);
    this->dataPtr->cubeCam[i]->yaw(Ogre::Degree(-90));
    this->dataPtr->cubeCam[i]->roll(Ogre::Degree(-90));

    // orient camera to create cubemap
    if (i == 0)
      this->dataPtr->cubeCam[i]->yaw(Ogre::Degree(-90));
    else if (i == 1)
      this->dataPtr->cubeCam[i]->yaw(Ogre::Degree(90));
    else if (i == 2)
      this->dataPtr->cubeCam[i]->pitch(Ogre::Degree(90));
    else if (i == 3)
      this->dataPtr->cubeCam[i]->pitch(Ogre::Degree(-90));
    else if (i == 5)
      this->dataPtr->cubeCam[i]->yaw(Ogre::Degree(180));

    // create render texture - these textures pack the range data
    // that will be used in the 2nd pass
    std::stringstream texName;
    texName << this->Name() << "_first_pass_" << i;
    this->dataPtr->firstPassTextures[i] =
      Ogre::TextureManager::getSingleton().createManual(
      texName.str(), "General", Ogre::TEX_TYPE_2D,
      this->dataPtr->w1st, this->dataPtr->h1st, 1, 0,
      Ogre::PF_FLOAT32_RGB, Ogre::TU_RENDERTARGET,
      0, false, 0, Ogre::BLANKSTRING, false, true);

    Ogre::RenderTarget *rt =
        this->dataPtr->firstPassTextures[i]->getBuffer()->getRenderTarget();
    // create compositor worksspace
    this->dataPtr->ogreCompositorWorkspace1st[i] =
        ogreCompMgr->addWorkspace(this->scene->OgreSceneManager(),
        rt, this->dataPtr->cubeCam[i], wsDefName, false);

    Ogre::CompositorNode *node =
        this->dataPtr->ogreCompositorWorkspace1st[i]->getNodeSequence()[0];
    auto channelsTex = node->getLocalTextures();

    for (auto c : channelsTex)
    {
      if (c.textures[0]->getSrcFormat() == Ogre::PF_R8G8B8)
      {
        // add laser retro material switcher to render target listener
        // so we can switch to use laser retro material when the camera is being
        // updated
        this->dataPtr->laserRetroMaterialSwitcher[i].reset(
            new Ogre2LaserRetroMaterialSwitcher(this->scene));
        c.target->addListener(
            this->dataPtr->laserRetroMaterialSwitcher[i].get());

        // add particle noise / scatter effects listener so we can set the
        // amount of noise based on size of emitter
        this->dataPtr->particleNoiseListener[i].reset(
            new Ogre2ParticleNoiseListener(this->scene,
            this->dataPtr->cubeCam[i], this->dataPtr->matFirstPass));
        c.target->addListener(this->dataPtr->particleNoiseListener[i].get());
        break;
      }
    }
  }
}

/////////////////////////////////////////////////////////
void Ogre2GpuRays::Setup2ndPass()
{
  // Create second pass RTT, which stores the final range data output
  // see PostRender on how we retrieve data from this texture
  this->dataPtr->secondPassTexture =
      Ogre::TextureManager::getSingleton().createManual(
      this->Name() + "_second_pass",
      "General",
      Ogre::TEX_TYPE_2D,
      this->dataPtr->w2nd, this->dataPtr->h2nd, 0,
      Ogre::PF_FLOAT32_RGB,
      Ogre::TU_RENDERTARGET);

  // Create second pass material
  // The GpuRaysScan2nd material is defined in script (gpu_rays.material).
  // We need to clone it since we are going to modify texture unit states.
  std::string mat2ndName = "GpuRaysScan2nd";
  Ogre::MaterialPtr mat2nd =
      Ogre::MaterialManager::getSingleton().getByName(mat2ndName);
  this->dataPtr->matSecondPass = mat2nd->clone(
      this->Name() + "_" + mat2ndName);
  this->dataPtr->matSecondPass->load();
  Ogre::Pass *pass = this->dataPtr->matSecondPass->getTechnique(0)->getPass(0);

  // Connect cubeUVTexture to the GpuRaysScan2nd material's texture unit state
  // The texture unit index (0) must match the one specified in the script
  // See GpuRaysScan2nd definition
  pass->getTextureUnitState(0)->setTexture(this->dataPtr->cubeUVTexture);

  // connect all cubemap textures to the corresponding texture unit states
  // defined in the GpuRaysScan2nd material
  Ogre::TextureUnitState *texUnit = nullptr;
  for (auto i : this->dataPtr->cubeFaceIdx)
  {
    // texIndex need to match how the texture units are defined in the
    // gpu_rays.material script
    unsigned int texIndex = 1 + i;
    texUnit = pass->getTextureUnitState(texIndex);
    texUnit->setTexture(this->dataPtr->firstPassTextures[i]);
  }

  // create 2nd pass compositor
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  // Same as 1st pass. We need to programmatically create the compositor in
  // order to configure it to use the cloned 2nd pass material created earlier
  // The compositor workspace definition is equivalent to the following
  // compositor script:
  //
  // compositor_node GpuRays2ndPass
  // {
  //   in 0 rt_input
  //   target rt_input
  //   {
  //     pass clear
  //     {
  //       colour_value 0.0 0.0 0.0 1.0
  //     }
  //     pass render_quad
  //     {
  //       material GpuRaysScan2nd // Use copy instead of original
  //     }
  //   }
  //   out 0 rt_input
  // }
  std::string wsDefName = "GpuRays2ndPassWorkspace_" + this->Name();
  this->dataPtr->ogreCompositorWorkspaceDef2nd = wsDefName;
  if (!ogreCompMgr->hasWorkspaceDefinition(wsDefName))
  {
    std::string nodeDefName = wsDefName + "/Node";
    this->dataPtr->ogreCompositorNodeDef2nd = nodeDefName;
    Ogre::CompositorNodeDef *nodeDef =
        ogreCompMgr->addNodeDefinition(nodeDefName);
    // Input texture
    nodeDef->addTextureSourceName("rt_input", 0,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    nodeDef->setNumTargetPass(1);
    {
      Ogre::CompositorTargetDef *inputTargetDef =
          nodeDef->addTargetPass("rt_input");
      inputTargetDef->setNumPasses(2);
      // clear pass
      Ogre::CompositorPassClearDef *passClear =
          static_cast<Ogre::CompositorPassClearDef *>(
          inputTargetDef->addPass(Ogre::PASS_CLEAR));
      passClear->mColourValue = Ogre::ColourValue(this->dataMaxVal, 0, 1.0);
      // quad pass - sample from cubemap textures
      Ogre::CompositorPassQuadDef *passQuad =
          static_cast<Ogre::CompositorPassQuadDef *>(
          inputTargetDef->addPass(Ogre::PASS_QUAD));
      passQuad->mMaterialName = this->dataPtr->matSecondPass->getName();
    }
    nodeDef->mapOutputChannel(0, "rt_input");

    Ogre::CompositorWorkspaceDef *workDef =
        ogreCompMgr->addWorkspaceDefinition(wsDefName);
    workDef->connectExternal(0, nodeDef->getName(), 0);
  }
  Ogre::CompositorWorkspaceDef *wsDef =
      ogreCompMgr->getWorkspaceDefinition(wsDefName);
  if (!wsDef)
  {
    ignerr << "Unable to add workspace definition [" << wsDefName << "] "
           << " for " << this->Name();
  }

  // create the compositor workspace
  Ogre::RenderTarget *rt =
      this->dataPtr->secondPassTexture->getBuffer()->getRenderTarget();
  this->dataPtr->ogreCompositorWorkspace2nd =
      ogreCompMgr->addWorkspace(this->scene->OgreSceneManager(),
      rt, this->dataPtr->ogreCamera, wsDefName, false);
}

/////////////////////////////////////////////////////////
void Ogre2GpuRays::CreateGpuRaysTextures()
{
  this->ConfigureCamera();
  this->CreateSampleTexture();
  this->Setup1stPass();
  this->Setup2ndPass();
}

/////////////////////////////////////////////////
void Ogre2GpuRays::UpdateRenderTarget1stPass()
{
  // update the compositors
  for (auto i : this->dataPtr->cubeFaceIdx)
    this->dataPtr->ogreCompositorWorkspace1st[i]->setEnabled(true);
  auto engine = Ogre2RenderEngine::Instance();
  engine->OgreRoot()->renderOneFrame();
  for (auto i : this->dataPtr->cubeFaceIdx)
    this->dataPtr->ogreCompositorWorkspace1st[i]->setEnabled(false);
}

/////////////////////////////////////////////////
void Ogre2GpuRays::UpdateRenderTarget2ndPass()
{
  this->dataPtr->ogreCompositorWorkspace2nd->setEnabled(true);
  auto engine = Ogre2RenderEngine::Instance();
  engine->OgreRoot()->renderOneFrame();
  this->dataPtr->ogreCompositorWorkspace2nd->setEnabled(false);
}

//////////////////////////////////////////////////
void Ogre2GpuRays::Render()
{
  this->UpdateRenderTarget1stPass();
  this->UpdateRenderTarget2ndPass();
}

//////////////////////////////////////////////////
void Ogre2GpuRays::PreRender()
{
  if (!this->dataPtr->cubeUVTexture)
    this->CreateGpuRaysTextures();
}

//////////////////////////////////////////////////
void Ogre2GpuRays::PostRender()
{
  unsigned int width = this->dataPtr->w2nd;
  unsigned int height = this->dataPtr->h2nd;

  size_t size = Ogre::PixelUtil::getMemorySize(
    width, height, 1, Ogre::PF_FLOAT32_RGB);
  int len = width * height * this->Channels();

  if (!this->dataPtr->gpuRaysBuffer)
  {
    this->dataPtr->gpuRaysBuffer = new float[len];
  }
  Ogre::PixelBox dstBox(width, height,
        1, Ogre::PF_FLOAT32_RGB, this->dataPtr->gpuRaysBuffer);

  // blit data from gpu to cpu
  auto rt = this->dataPtr->secondPassTexture->getBuffer()->getRenderTarget();
  rt->copyContentsToMemory(dstBox, Ogre::RenderTarget::FB_FRONT);

  if (!this->dataPtr->gpuRaysScan)
  {
    this->dataPtr->gpuRaysScan = new float[len];
  }

  memcpy(this->dataPtr->gpuRaysScan, this->dataPtr->gpuRaysBuffer, size);

  this->dataPtr->newGpuRaysFrame(this->dataPtr->gpuRaysScan,
      width, height, this->Channels(), "PF_FLOAT32_RGB");

  // Uncomment to debug output
  // igndbg << "wxh: " << width << " x " << height << std::endl;
  // for (unsigned int i = 0; i < height; ++i)
  // {
  //   for (unsigned int j = 0; j < width; ++j)
  //   {
  //     igndbg << "[" << this->dataPtr->gpuRaysBuffer[i*width*3 + j*3] <<  " ";
  //     igndbg << this->dataPtr->gpuRaysBuffer[i*width*3 + j*3 + 1] <<  " ";
  //     igndbg << this->dataPtr->gpuRaysBuffer[i*width*3 + j*3 + 2] <<  "] ";
  //   }
  //   igndbg << std::endl;
  // }
}

//////////////////////////////////////////////////
const float* Ogre2GpuRays::Data() const
{
  return this->dataPtr->gpuRaysScan;
}

//////////////////////////////////////////////////
void Ogre2GpuRays::Copy(float *_dataDest)
{
  unsigned int width = this->dataPtr->w2nd;
  unsigned int height = this->dataPtr->h2nd;

  size_t size = Ogre::PixelUtil::getMemorySize(
    width, height, 1, Ogre::PF_FLOAT32_RGB);

  memcpy(_dataDest, this->dataPtr->gpuRaysScan, size);
}

/////////////////////////////////////////////////
void Ogre2GpuRays::Set1stTextureSize(
    const unsigned int _w, const unsigned int _h)
{
  this->dataPtr->w1st = _w;
  this->dataPtr->h1st = _h;
}

/////////////////////////////////////////////////
void Ogre2GpuRays::SetRangeCount(
    const unsigned int _w, const unsigned int _h)
{
  this->dataPtr->w2nd = _w;
  this->dataPtr->h2nd = _h;
}

//////////////////////////////////////////////////
ignition::common::ConnectionPtr Ogre2GpuRays::ConnectNewGpuRaysFrame(
    std::function<void(const float *_frame, unsigned int _width,
    unsigned int _height, unsigned int _channels,
    const std::string &/*_format*/)> _subscriber)
{
  return this->dataPtr->newGpuRaysFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
RenderTargetPtr Ogre2GpuRays::RenderTarget() const
{
  return this->dataPtr->renderTexture;
}
