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

// // leave this out of OgreIncludes as it conflicts with other files requiring
// // gl.h
// #ifdef _MSC_VER
// #pragma warning(push, 0)
// #endif
// #include <OgreGL3PlusFBORenderTexture.h>
// #ifdef _MSC_VER
// #pragma warning(pop)
// #endif

#include <ignition/common/Console.hh>

#include "ignition/rendering/Material.hh"

#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2RenderPass.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2Material.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
//
/// \brief Listener for changing ogre compositor pass properties
class Ogre2RenderTargetCompositorListener :
    public Ogre::CompositorWorkspaceListener
{
  /// \brief Constructor
  /// \param[in] _target ogre render target object
  public: explicit Ogre2RenderTargetCompositorListener(
      Ogre2RenderTarget *_target)
  {
    this->ogreRenderTarget = _target;
  }

  /// \brief Destructor
  public: virtual ~Ogre2RenderTargetCompositorListener() = default;

  // Documentation inherited.
  public: virtual void passPreExecute(Ogre::CompositorPass *_pass)
  {
    if (_pass->getType() == Ogre::PASS_SCENE)
    {
      Ogre::CompositorPassScene *scenePass =
          static_cast<Ogre::CompositorPassScene *>(_pass);
      IGN_ASSERT(scenePass != nullptr, "Unable to get scene pass");
      Ogre::Camera *vp = scenePass->getCamera();
      // make sure we do not alter the reserved visibility flags
      uint32_t f = this->ogreRenderTarget->VisibilityMask() |
          ~Ogre::VisibilityFlags::RESERVED_VISIBILITY_FLAGS;
      // apply the new visibility mask
      uint32_t flags = f & vp->getVisibilityFlags();
      vp->setVisibilityFlags(flags);
    }
  }

  /// \brief Pointer to render target that added this listener
  private: Ogre2RenderTarget *ogreRenderTarget = nullptr;
};
}
}
}

/// \brief Private data class for Ogre2RenderTarget
class ignition::rendering::Ogre2RenderTargetPrivate
{
  /// \brief Listener for chaning compositor pass properties
  public: Ogre2RenderTargetCompositorListener *rtListener = nullptr;

  /// \brief Name of sky box material
  public: const std::string kSkyboxMaterialName = "SkyBox";

  /// \brief Name of base rendering compositor node
  public: const std::string kBaseNodeName = "PbsMaterialsRenderingNode";

  /// \brief Name of final rendering compositor node
  public: const std::string kFinalNodeName = "FinalComposition";

  /// \brief Name of shadow compositor node
  public: const std::string kShadowNodeName = "PbsMaterialsShadowNode";
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
// Ogre2RenderTarget
//////////////////////////////////////////////////
Ogre2RenderTarget::Ogre2RenderTarget()
  : dataPtr(new Ogre2RenderTargetPrivate)
{
  this->ogreBackgroundColor = Ogre::ColourValue::Black;
  this->ogreCompositorWorkspaceDefName = "PbsMaterialsWorkspace";
}

//////////////////////////////////////////////////
Ogre2RenderTarget::~Ogre2RenderTarget()
{
  // if (this->dataPtr->rtListener)
  // {
  //   delete this->dataPtr->rtListener;
  //   this->dataPtr->rtListener = nullptr;
  // }
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::BuildCompositor()
{
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  this->UpdateBackgroundMaterial();

  bool validBackground = this->backgroundMaterial &&
      !this->backgroundMaterial->EnvironmentMap().empty();
  //
  // // The function build a similar compositor as the one defined in
  // // ogre2/media/2.0/scripts/Compositors/PbsMaterials.compositor
  // // but supports an extra quad pass to render the skybox cubemap if
  // // sky is enabled.
  // // todo(anyone) Note the definition programmatically created here
  // // replaces the one defined in the script so it maybe safe to remove the
  // // PbsMaterials.compositor file
  std::string wsDefName = "PbsMaterialWorkspace_" + this->Name();
  this->ogreCompositorWorkspaceDefName = wsDefName;
  if (!ogreCompMgr->hasWorkspaceDefinition(wsDefName))
  {
    // PbsMaterialsRenderingNode
    std::string nodeDefName = wsDefName + "/" + this->dataPtr->kBaseNodeName;
    Ogre::CompositorNodeDef *nodeDef =
        ogreCompMgr->addNodeDefinition(nodeDefName);

    // Input texture
    Ogre::TextureDefinitionBase::TextureDefinition *rt0TexDef =
        nodeDef->addTextureDefinition("rt0");
    rt0TexDef->textureType = Ogre::TextureTypes::Type2D;
    rt0TexDef->width = 0;
    rt0TexDef->height = 0;
    rt0TexDef->depthOrSlices = 1;
    rt0TexDef->numMipmaps = 0;
    rt0TexDef->widthFactor = 1;
    rt0TexDef->heightFactor = 1;
    rt0TexDef->format = Ogre::PFG_RGBA8_UNORM;
    rt0TexDef->textureFlags |= !Ogre::TextureFlags::Uav;
    rt0TexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
    rt0TexDef->depthBufferFormat = Ogre::PFG_UNKNOWN;

    Ogre::RenderTargetViewDef *rtv = nodeDef->addRenderTextureView("rt0");
    rtv->setForTextureDefinition("rt0", rt0TexDef);

    Ogre::TextureDefinitionBase::TextureDefinition *rt1TexDef =
        nodeDef->addTextureDefinition("rt1");
    rt1TexDef->textureType = Ogre::TextureTypes::Type2D;
    rt1TexDef->width = 0;
    rt1TexDef->height = 0;
    rt1TexDef->depthOrSlices = 1;
    rt1TexDef->widthFactor = 1;
    rt1TexDef->heightFactor = 1;
    rt1TexDef->format = Ogre::PFG_RGBA8_UNORM;
    rt1TexDef->textureFlags |= !Ogre::TextureFlags::Uav;
    rt1TexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
    rt1TexDef->depthBufferFormat = Ogre::PFG_UNKNOWN;

    Ogre::RenderTargetViewDef *rtv2 = nodeDef->addRenderTextureView("rt1");
    rtv2->setForTextureDefinition("rt1", rt1TexDef);

    nodeDef->setNumTargetPass(2);
    Ogre::CompositorTargetDef *rt0TargetDef =
        nodeDef->addTargetPass("rt0");

    if (validBackground)
      rt0TargetDef->setNumPasses(3);
    else
      rt0TargetDef->setNumPasses(2);
    {
      // clear pass
      Ogre::CompositorPassClearDef *passClear =
          static_cast<Ogre::CompositorPassClearDef *>(
          rt0TargetDef->addPass(Ogre::PASS_CLEAR));
      passClear->setAllClearColours(this->ogreBackgroundColor);

      if (validBackground)
      {
        // quad pass
        Ogre::CompositorPassQuadDef *passQuad =
            static_cast<Ogre::CompositorPassQuadDef *>(
            rt0TargetDef->addPass(Ogre::PASS_QUAD));
        passQuad->mMaterialName = this->dataPtr->kSkyboxMaterialName + "_"
            + this->Name();
        passQuad->mFrustumCorners =
            Ogre::CompositorPassQuadDef::CAMERA_DIRECTION;
      }

      // scene pass
      Ogre::CompositorPassSceneDef *passScene =
          static_cast<Ogre::CompositorPassSceneDef *>(
          rt0TargetDef->addPass(Ogre::PASS_SCENE));
      passScene->mShadowNode = this->dataPtr->kShadowNodeName;
      passScene->mIncludeOverlays = true;
    }

    nodeDef->mapOutputChannel(0, "rt0");
    nodeDef->mapOutputChannel(1, "rt1");

    // Final Composition
    std::string finalNodeDefName = wsDefName + "/" +
        this->dataPtr->kFinalNodeName;
    Ogre::CompositorNodeDef *finalNodeDef =
        ogreCompMgr->addNodeDefinition(finalNodeDefName);
    finalNodeDef->addTextureSourceName("rt_output", 0,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    finalNodeDef->addTextureSourceName("rtN", 1,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);

    finalNodeDef->setNumTargetPass(2);
    Ogre::CompositorTargetDef *outTargetDef =
        finalNodeDef->addTargetPass("rt_output");
    outTargetDef->setNumPasses(2);
    {
      // quad pass
      Ogre::CompositorPassQuadDef *passQuad =
          static_cast<Ogre::CompositorPassQuadDef *>(
          outTargetDef->addPass(Ogre::PASS_QUAD));
      passQuad->mMaterialName = "Ogre/Copy/4xFP32";
      passQuad->addQuadTextureSource(0, "rtN");

      // scene pass
      Ogre::CompositorPassSceneDef *passScene =
          static_cast<Ogre::CompositorPassSceneDef *>(
          outTargetDef->addPass(Ogre::PASS_SCENE));
      passScene->mUpdateLodLists = false;
      passScene->mIncludeOverlays = true;
      passScene->mFirstRQ = 254;
      passScene->mLastRQ = 255;
    }

    Ogre::CompositorWorkspaceDef *workDef =
        ogreCompMgr->addWorkspaceDefinition(wsDefName);
    workDef->connectExternal(0, finalNodeDefName, 0);
    workDef->connect(nodeDefName, 0, finalNodeDefName, 1);
  }

  try{
    std::cerr << "/* PbsMaterialWs workspace */" << '\n';

    this->UpdateShadowNode();

    this->ogreCompositorWorkspace =
        ogreCompMgr->addWorkspace(
          this->scene->OgreSceneManager(),
          engine->getWindow()->getTexture(),
          this->ogreCamera,
          this->ogreCompositorWorkspaceDefName, true);

    // const Ogre::String workspaceName = "ShadowWorkspace";
    const Ogre::String nodeDefName = "AutoGen " +
                                     Ogre::IdString(this->ogreCompositorWorkspaceDefName +
                                                    "/Node").getReleaseText();
    if (ogreCompMgr->hasNodeDefinition(nodeDefName))
    {
      Ogre::CompositorNodeDef *nodeDef =
              ogreCompMgr->getNodeDefinitionNonConst( nodeDefName );

      Ogre::CompositorTargetDef *targetDef = nodeDef->getTargetPass( 0 );
      const Ogre::CompositorPassDefVec &passes = targetDef->getCompositorPasses();

      assert( dynamic_cast<Ogre::CompositorPassSceneDef*>( passes[0] ) );
      Ogre::CompositorPassSceneDef *passSceneDef =
              static_cast<Ogre::CompositorPassSceneDef*>( passes[0] );
      passSceneDef->mShadowNode = "PbsMaterialsShadowNode";
    }
  } catch (Ogre::Exception &e)
  {
    ignerr << "Ogre Error Ogre2RenderTexture::BuildCompositor: " << e.getFullDescription() << "\n";
  }
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::DestroyCompositor()
{
  if (!this->ogreCompositorWorkspace)
    return;

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();
  this->ogreCompositorWorkspace->addListener(nullptr);
  ogreCompMgr->removeWorkspace(this->ogreCompositorWorkspace);
  ogreCompMgr->removeWorkspaceDefinition(this->ogreCompositorWorkspaceDefName);
  ogreCompMgr->removeNodeDefinition(this->ogreCompositorWorkspaceDefName +
      "/" + this->dataPtr->kBaseNodeName);
  ogreCompMgr->removeNodeDefinition(this->ogreCompositorWorkspaceDefName +
      "/" + this->dataPtr->kFinalNodeName);

  this->ogreCompositorWorkspace = nullptr;
  // delete this->dataPtr->rtListener;
  // this->dataPtr->rtListener = nullptr;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::RebuildCompositor()
{
  this->DestroyCompositor();
  this->BuildCompositor();
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::Copy(Image &_image) const
{
  // TODO(anyone) handle Bayer conversions
  // TODO(anyone) handle ogre version differences

  if (_image.Width() != this->width || _image.Height() != this->height)
  {
    ignerr << "Invalid image dimensions" << std::endl;
    return;
  }

  auto engine = Ogre2RenderEngine::Instance();
  auto textureWindow = engine->getWindow()->getTexture();

  Ogre::Image2 image2;
  image2.convertFromTexture( textureWindow, 0u, 0u );

  auto dataImage = static_cast<unsigned char *>(_image.Data());
  auto dataImage2 = static_cast<Ogre::uint8 *>(image2.getRawBuffer());

  for (int row = 0; row < textureWindow->getHeight(); ++row)
  {
    for (int column = 0; column < textureWindow->getWidth(); ++column)
    {
      dataImage[3 * (textureWindow->getHeight() * row + column)] = dataImage2[4 * (textureWindow->getHeight() * row + column)];
      dataImage[3 * (textureWindow->getHeight() * row + column) + 1] = dataImage2[4 * (textureWindow->getHeight() * row + column) + 1];
      dataImage[3 * (textureWindow->getHeight() * row + column) + 2] = dataImage2[4 * (textureWindow->getHeight() * row + column) + 2];
    }
  }
}

//////////////////////////////////////////////////
Ogre::Camera *Ogre2RenderTarget::Camera() const
{
  return this->ogreCamera;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetCamera(Ogre::Camera *_camera)
{
  this->ogreCamera = _camera;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
math::Color Ogre2RenderTarget::BackgroundColor() const
{
  return Ogre2Conversions::Convert(this->ogreBackgroundColor);
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetBackgroundColor(math::Color _color)
{
  this->ogreBackgroundColor = Ogre2Conversions::Convert(_color);
  this->colorDirty = true;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetBackgroundMaterial(MaterialPtr _material)
{
  this->backgroundMaterial = _material;
  this->backgroundMaterialDirty = true;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
MaterialPtr Ogre2RenderTarget::BackgroundMaterial() const
{
  return this->backgroundMaterial;
}

//////////////////////////////////////////////////
unsigned int Ogre2RenderTarget::AntiAliasing() const
{
  return this->antiAliasing;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetAntiAliasing(unsigned int _aa)
{
  this->antiAliasing = _aa;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::PreRender()
{
  BaseRenderTarget::PreRender();
  this->UpdateBackgroundColor();

  if (this->material)
  {
    this->material->PreRender();
  }

  this->UpdateRenderPassChain();
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::PostRender()
{
  // do nothing by default
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::Render()
{
  // TODO(anyone)
  // There is current not an easy solution to manually updating
  // render textures:
  // https://forums.ogre3d.org/viewtopic.php?t=84687
  this->ogreCompositorWorkspace->setEnabled(true);
  // this->ogreCompositorWorkspace2->setEnabled(true);
  auto engine = Ogre2RenderEngine::Instance();
  Ogre::WindowEventUtilities::messagePump();
  engine->OgreRoot()->renderOneFrame();
  // this->ogreCompositorWorkspace2->setEnabled(false);
  this->ogreCompositorWorkspace->setEnabled(false);

  // The code below for manual updating render textures was suggested in ogre
  // forum but it does not seem to work
  // this->scene->OgreSceneManager()->updateSceneGraph();
  // this->ogreCompositorWorkspace->_validateFinalTarget();
  // engine->OgreRoot()->getRenderSystem()->_beginFrameOnce();
  // this->ogreCompositorWorkspace->_beginUpdate(false);
  // this->ogreCompositorWorkspace->_update();
  // this->ogreCompositorWorkspace->_endUpdate(false);

  // this->scene->OgreSceneManager()->_frameEnded();
  // for (size_t i=0; i < Ogre::HLMS_MAX; ++i)
  // {
  //   Ogre::Hlms *hlms = engine->OgreRoot()->getHlmsManager()->getHlms(
  //       static_cast<Ogre::HlmsTypes>(i));
  //   if(hlms)
  //     hlms->frameEnded();
  // }
  // engine->OgreRoot()->getRenderSystem()->_update();
}

//////////////////////////////////////////////////
uint32_t Ogre2RenderTarget::VisibilityMask() const
{
  return this->visibilityMask;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetVisibilityMask(uint32_t _mask)
{
  this->visibilityMask = _mask;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateBackgroundColor()
{
  if (this->colorDirty)
  {
    if(!this->ogreCompositorWorkspace)
      return;

    // set background color in compositor clear pass def
    auto nodeSeq = this->ogreCompositorWorkspace->getNodeSequence();
    auto pass = nodeSeq[0]->_getPasses()[0]->getDefinition();
    auto clearPass = dynamic_cast<const Ogre::CompositorPassClearDef *>(pass);
    // TODO(ahcorde)
    const_cast<Ogre::CompositorPassClearDef *>(clearPass)->mClearColour[0] =
        this->ogreBackgroundColor;

    this->colorDirty = false;
  }
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateBackgroundMaterial()
{
  if (!this->backgroundMaterialDirty)
    return;

  bool validBackground = this->backgroundMaterial &&
      !this->backgroundMaterial->EnvironmentMap().empty();

  if (validBackground)
  {
    Ogre::MaterialManager &matManager = Ogre::MaterialManager::getSingleton();
    std::string skyMatName = this->dataPtr->kSkyboxMaterialName + "_"
        + this->Name();
    auto mat = matManager.getByName(skyMatName);
    if (!mat)
    {
      auto skyboxMat = matManager.getByName(this->dataPtr->kSkyboxMaterialName);
      if (!skyboxMat)
      {
        ignerr << "Unable to find skybox material" << std::endl;
        return;
      }
      mat = skyboxMat->clone(skyMatName);
    }
    Ogre::TextureUnitState *texUnit =
        mat->getTechnique(0u)->getPass(0u)->getTextureUnitState(0u);
    texUnit->setTextureName(this->backgroundMaterial->EnvironmentMap(),
        Ogre::TextureTypes::TypeCube);
  }

  this->backgroundMaterialDirty = false;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateRenderPassChain()
{
  UpdateRenderPassChain(this->ogreCompositorWorkspace,
      this->ogreCompositorWorkspaceDefName,
      this->ogreCompositorWorkspaceDefName + "/" +
      this->dataPtr->kBaseNodeName,
      this->ogreCompositorWorkspaceDefName + "/" +
      this->dataPtr->kFinalNodeName,
      this->renderPasses, this->renderPassDirty);

  this->renderPassDirty = false;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateRenderPassChain(
    Ogre::CompositorWorkspace *_workspace, const std::string &_workspaceDefName,
    const std::string &_baseNode, const std::string &_finalNode,
    const std::vector<RenderPassPtr> &_renderPasses,
    bool _recreateNodes)
{
  if (!_workspace || _workspaceDefName.empty() ||
      _baseNode.empty() || _finalNode.empty() || _renderPasses.empty())
    return;

  // check pass enabled state and update connections if necessary.
  // If render pass is dirty then skip the enabled state check since the whole
  // workspace nodes and connections will be recreated
  bool updateConnection = false;
  if (!_recreateNodes)
  {
    auto nodeSeq = _workspace->getNodeSequence();

    // set node instance to render pass and update enabled state
    for (const auto &pass : _renderPasses)
    {
      Ogre2RenderPass *ogre2RenderPass =
          dynamic_cast<Ogre2RenderPass *>(pass.get());
      Ogre::CompositorNode *node =
          _workspace->findNodeNoThrow(
          ogre2RenderPass->OgreCompositorNodeDefinitionName());

      // check if we need to create all nodes or just update the connections.
      // if node does not exist then it means it has not been added to the
      // chain yet, in which case, we need to recreate the nodes and
      // connections
      if (!node)
      {
        _recreateNodes = true;
      }
      else if (node && node->getEnabled() != ogre2RenderPass->IsEnabled())
      {
        node->setEnabled(ogre2RenderPass->IsEnabled());
        updateConnection = true;
      }
    }
  }

  if (!_recreateNodes && !updateConnection)
    return;

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  Ogre::CompositorWorkspaceDef *workspaceDef =
    ogreCompMgr->getWorkspaceDefinition(_workspaceDefName);

  auto nodeSeq = _workspace->getNodeSequence();

  // The first node and final node in the workspace are defined in
  // PbsMaterials.compositor
  // the first node is the base scene pass node
  std::string outNodeDefName = _baseNode;
  // the final compositor node
  std::string finalNodeDefName = _finalNode;
  std::string inNodeDefName;

  // if new nodes need to be added then clear everything,
  // otherwise clear only the node connections
  if (_recreateNodes)
    workspaceDef->clearAll();
  else
    workspaceDef->clearAllInterNodeConnections();

  // chain the render passes by connecting all the ogre compositor nodes
  // in between the base scene pass node and the final compositor node
  for (const auto &pass : _renderPasses)
  {
    Ogre2RenderPass *ogre2RenderPass =
        dynamic_cast<Ogre2RenderPass *>(pass.get());
    ogre2RenderPass->CreateRenderPass();
    inNodeDefName = ogre2RenderPass->OgreCompositorNodeDefinitionName();
    // only connect passes that are enabled
    if (!inNodeDefName.empty() && ogre2RenderPass->IsEnabled())
    {
      workspaceDef->connect(outNodeDefName, inNodeDefName);
      outNodeDefName = inNodeDefName;
    }
  }

  // connect the last render pass to the final compositor node
  workspaceDef->connect(outNodeDefName, 0,  finalNodeDefName, 1);

  // if new node definitions were added then recreate all the compositor nodes,
  // otherwise update the connections
  if (_recreateNodes)
  {
    // clearAll requires the output to be connected again.
    workspaceDef->connectExternal(0, finalNodeDefName, 0);
    _workspace->recreateAllNodes();
  }
  else
  {
    _workspace->reconnectAllNodes();
  }
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateShadowNode()
{
  if (!this->scene->ShadowsDirty())
    return;

  unsigned int spotPointLightCount = 0;
  unsigned int dirLightCount = 0;

  for (unsigned int i = 0; i < this->scene->LightCount(); ++i)
  {
    LightPtr light = this->scene->LightByIndex(i);
    if (light->CastShadows())
    {
      if (std::dynamic_pointer_cast<DirectionalLight>(light))
        dirLightCount++;
      else
        spotPointLightCount++;
    }
  }

  // limit number of shadow maps
  // shaders dynamically generated by ogre produce compile error at runtime if
  // the number of shadow maps exceeds certain number. The error seems to
  // suggest that the number of uniform variables has exceeded the max number
  // allowed
  unsigned int maxShadowMaps = 25u;
  if (dirLightCount * 3 + spotPointLightCount > maxShadowMaps)
  {
    dirLightCount = std::min(static_cast<unsigned int>(maxShadowMaps / 3),
        dirLightCount);
    spotPointLightCount = std::min(
        std::max(maxShadowMaps - dirLightCount * 3, 0u), spotPointLightCount);
    ignwarn << "Number of shadow-casting lights exceeds the limit supported by "
            << "the underlying rendering engine ogre2. Limiting to "
            << dirLightCount << " directional lights and "
            << spotPointLightCount << " point / spot lights" << std::endl;
  }

  auto engine = Ogre2RenderEngine::Instance();
  Ogre::CompositorManager2 *compositorManager =
      engine->OgreRoot()->getCompositorManager2();

  Ogre::ShadowNodeHelper::ShadowParamVec shadowParams;
  Ogre::ShadowNodeHelper::ShadowParam shadowParam;

  // directional lights
  unsigned int atlasId = 0u;
  unsigned int texSize = 2048u;
  unsigned int halfTexSize = texSize * 0.5;
  for (unsigned int i = 0; i < dirLightCount; ++i)
  {
    shadowParam.technique = Ogre::SHADOWMAP_PSSM;
    shadowParam.atlasId = atlasId;
    shadowParam.numPssmSplits = 3u;
    shadowParam.resolution[0].x = texSize;
    shadowParam.resolution[0].y = texSize;
    shadowParam.resolution[1].x = halfTexSize;
    shadowParam.resolution[1].y = halfTexSize;
    shadowParam.resolution[2].x = halfTexSize;
    shadowParam.resolution[2].y = halfTexSize;
    shadowParam.atlasStart[0].x = 0u;
    shadowParam.atlasStart[0].y = 0u;
    shadowParam.atlasStart[1].x = 0u;
    shadowParam.atlasStart[1].y = texSize;
    shadowParam.atlasStart[2].x = halfTexSize;
    shadowParam.atlasStart[2].y = texSize;
    shadowParam.supportedLightTypes = 0u;
    shadowParam.addLightType(Ogre::Light::LT_DIRECTIONAL);
    shadowParams.push_back(shadowParam);
    atlasId++;
  }

  // others
  unsigned int maxTexSize = 8192u;
  unsigned int rowIdx = 0;
  unsigned int colIdx = 0;
  unsigned int rowSize = maxTexSize / texSize;
  unsigned int colSize = rowSize;

  for (unsigned int i = 0; i < spotPointLightCount; ++i)
  {
    shadowParam.technique = Ogre::SHADOWMAP_FOCUSED;
    shadowParam.atlasId = atlasId;
    shadowParam.resolution[0].x = texSize;
    shadowParam.resolution[0].y = texSize;
    shadowParam.atlasStart[0].x = colIdx * texSize;
    shadowParam.atlasStart[0].y = rowIdx * texSize;

    shadowParam.supportedLightTypes = 0u;
    shadowParam.addLightType(Ogre::Light::LT_DIRECTIONAL);
    shadowParam.addLightType(Ogre::Light::LT_POINT);
    shadowParam.addLightType(Ogre::Light::LT_SPOTLIGHT);
    shadowParams.push_back(shadowParam);

    colIdx++;
    colIdx = colIdx % colSize;
    if (colIdx == 0u)
      rowIdx++;

    // check if we've filled the current texture atlas
    // if so, increment atlas id to indicate we want a new texture
    if (rowIdx >= rowSize)
    {
      atlasId++;
      colIdx = 0;
      rowIdx = 0;
    }
  }

  std::string shadowNodeDefName = this->dataPtr->kShadowNodeName;
  if (compositorManager->hasShadowNodeDefinition(shadowNodeDefName))
    compositorManager->removeShadowNodeDefinition(shadowNodeDefName);

  this->CreateShadowNodeWithSettings(compositorManager, shadowNodeDefName,
      shadowParams);

  this->scene->SetShadowsDirty(false);
}

////////////////////////////////////////////////////
void Ogre2RenderTarget::CreateShadowNodeWithSettings(
    Ogre::CompositorManager2 *_compositorManager,
    const std::string &_shadowNodeName,
    const Ogre::ShadowNodeHelper::ShadowParamVec &_shadowParams)
{
  Ogre::uint32 pointLightCubemapResolution = 1024u;
  Ogre::Real pssmLambda = 0.95f;
  Ogre::Real splitPadding = 1.0f;
  Ogre::Real splitBlend = 0.125f;
  Ogre::Real splitFade = 0.313f;

  const Ogre::uint32 spotMask           = 1u << Ogre::Light::LT_SPOTLIGHT;
  const Ogre::uint32 directionalMask    = 1u << Ogre::Light::LT_DIRECTIONAL;
  const Ogre::uint32 pointMask          = 1u << Ogre::Light::LT_POINT;
  const Ogre::uint32 spotAndDirMask = spotMask | directionalMask;

  typedef Ogre::vector<Ogre::ShadowNodeHelper::Resolution>::type ResolutionVec;

  size_t numExtraShadowMapsForPssmSplits = 0;
  size_t numTargetPasses = 0;
  ResolutionVec atlasResolutions;

  // Validation and data gathering
  bool hasPointLights = false;

  Ogre::ShadowNodeHelper::ShadowParamVec::const_iterator itor =
      _shadowParams.begin();
  Ogre::ShadowNodeHelper::ShadowParamVec::const_iterator end =
      _shadowParams.end();

  while (itor != end)
  {
    if (itor->technique == Ogre::SHADOWMAP_PSSM)
    {
      numExtraShadowMapsForPssmSplits = itor->numPssmSplits - 1u;
      // 1 per PSSM split
      numTargetPasses += numExtraShadowMapsForPssmSplits + 1u;
    }

    if (itor->atlasId >= atlasResolutions.size())
      atlasResolutions.resize(itor->atlasId + 1u);

    Ogre::ShadowNodeHelper::Resolution &resolution =
        atlasResolutions[itor->atlasId];

    const size_t numSplits = itor->technique == Ogre::SHADOWMAP_PSSM ?
        itor->numPssmSplits : 1u;
    for (size_t i = 0; i < numSplits; ++i)
    {
      resolution.x = std::max(resolution.x,
          itor->atlasStart[i].x + itor->resolution[i].x);
      resolution.y = std::max(resolution.y,
          itor->atlasStart[i].y + itor->resolution[i].y);
    }

    if (itor->supportedLightTypes & pointMask)
    {
      hasPointLights = true;
      // 6 target passes per cubemap + 1 for copy
      numTargetPasses += 7u;
    }
    if (itor->supportedLightTypes & spotAndDirMask &&
        itor->technique != Ogre::SHADOWMAP_PSSM)
    {
      // 1 per directional/spot light (for non-PSSM techniques)
      numTargetPasses += 1u;
    }
    ++itor;
  }

  // One clear for each atlas
  numTargetPasses += atlasResolutions.size();
  // Create the shadow node definition
  Ogre::CompositorShadowNodeDef *shadowNodeDef =
      _compositorManager->addShadowNodeDefinition(_shadowNodeName);

  const size_t numTextures = atlasResolutions.size();
  {
    // Define the atlases (textures)
    shadowNodeDef->setNumLocalTextureDefinitions(
        numTextures + (hasPointLights ? 1u : 0u));
    for (size_t i = 0; i < numTextures; ++i)
    {
      const Ogre::ShadowNodeHelper::Resolution &atlasRes = atlasResolutions[i];
      Ogre::String texName = "atlas" + Ogre::StringConverter::toString(i);
      Ogre::TextureDefinitionBase::TextureDefinition *texDef =
          shadowNodeDef->addTextureDefinition(
          "atlas" + Ogre::StringConverter::toString(i));

      texDef->width = std::max(atlasRes.x, 1u);
      texDef->height = std::max(atlasRes.y, 1u);
      texDef->format = Ogre::PFG_D32_FLOAT;
      texDef->depthBufferId = Ogre::DepthBuffer::POOL_NON_SHAREABLE;
      texDef->depthBufferFormat = Ogre::PFG_D32_FLOAT;
      texDef->preferDepthTexture = false;
      texDef->fsaa = "0";
      Ogre::RenderTargetViewDef *rtv = shadowNodeDef->addRenderTextureView( texName );
      rtv->setForTextureDefinition( texName, texDef );
    }

    // Define the cubemap needed by point lights
    if (hasPointLights)
    {
      Ogre::TextureDefinitionBase::TextureDefinition *texDef =
          shadowNodeDef->addTextureDefinition("tmpCubemap");

      texDef->width   = pointLightCubemapResolution;
      texDef->height  = pointLightCubemapResolution;
      texDef->depthOrSlices   = 6u;
      texDef->textureType = Ogre::TextureTypes::TypeCube;
      texDef->format = Ogre::PFG_R16_UNORM;
      texDef->depthBufferId = 1u;
      texDef->depthBufferFormat = Ogre::PFG_D32_FLOAT;
      texDef->preferDepthTexture = false;
      texDef->fsaa = "0";
      Ogre::RenderTargetViewDef *rtv = shadowNodeDef->addRenderTextureView( "tmpCubemap" );
      rtv->setForTextureDefinition( "tmpCubemap", texDef );
    }
  }

  // Create the shadow maps
  const size_t numShadowMaps =
      _shadowParams.size() + numExtraShadowMapsForPssmSplits;
  shadowNodeDef->setNumShadowTextureDefinitions(numShadowMaps);

  itor = _shadowParams.begin();

  while (itor != end)
  {
    const size_t lightIdx = itor - _shadowParams.begin();
    const Ogre::ShadowNodeHelper::ShadowParam &shadowParam = *itor;

    const Ogre::ShadowNodeHelper::Resolution &texResolution =
        atlasResolutions[shadowParam.atlasId];

    const size_t numSplits =
        shadowParam.technique == Ogre::SHADOWMAP_PSSM ?
        shadowParam.numPssmSplits : 1u;

    for (size_t j = 0; j < numSplits; ++j)
    {
      Ogre::Vector2 uvOffset(
          shadowParam.atlasStart[j].x, shadowParam.atlasStart[j].y);
      Ogre::Vector2 uvLength(
          shadowParam.resolution[j].x, shadowParam.resolution[j].y);

      uvOffset /= Ogre::Vector2(texResolution.x, texResolution.y);
      uvLength /= Ogre::Vector2(texResolution.x, texResolution.y);

      const Ogre::String texName =
          "atlas" + Ogre::StringConverter::toString(shadowParam.atlasId);

      Ogre::ShadowTextureDefinition *shadowTexDef =
          shadowNodeDef->addShadowTextureDefinition(lightIdx, j, texName,
          uvOffset, uvLength, 0);
      shadowTexDef->shadowMapTechnique = shadowParam.technique;
      shadowTexDef->pssmLambda = pssmLambda;
      shadowTexDef->splitPadding = splitPadding;
      shadowTexDef->splitBlend = splitBlend;
      shadowTexDef->splitFade = splitFade;
      shadowTexDef->numSplits = numSplits;
      shadowTexDef->numStableSplits = 0;
    }
    ++itor;
  }

  shadowNodeDef->setNumTargetPass(numTargetPasses);

  // Create the passes for each atlas
  for (size_t atlasId = 0; atlasId < numTextures; ++atlasId)
  {
    const Ogre::String texName =
        "atlas" + Ogre::StringConverter::toString(atlasId);
    {
      // Atlas clear pass
      Ogre::CompositorTargetDef *targetDef =
          shadowNodeDef->addTargetPass(texName);
      targetDef->setNumPasses(1u);

      Ogre::CompositorPassDef *passDef = targetDef->addPass(Ogre::PASS_CLEAR);
      Ogre::CompositorPassClearDef *passClear =
          static_cast<Ogre::CompositorPassClearDef *>(passDef);
      // TODO(ahcorde): review this, it's an array of 8 positions
      passClear->setAllClearColours(Ogre::ColourValue::White);
      passClear->mClearDepth = 1.0f;
    }

    // Pass scene for directional and spot lights first
    size_t shadowMapIdx = 0;
    itor = _shadowParams.begin();
    while (itor != end)
    {
      const Ogre::ShadowNodeHelper::ShadowParam &shadowParam = *itor;
      const size_t numSplits = shadowParam.technique == Ogre::SHADOWMAP_PSSM ?
          shadowParam.numPssmSplits : 1u;
      if (shadowParam.atlasId == atlasId &&
          shadowParam.supportedLightTypes & spotAndDirMask)
      {
        size_t currentShadowMapIdx = shadowMapIdx;
        for (size_t i = 0; i < numSplits; ++i)
        {
          Ogre::CompositorTargetDef *targetDef =
              shadowNodeDef->addTargetPass(texName);
          targetDef->setShadowMapSupportedLightTypes(
              shadowParam.supportedLightTypes & spotAndDirMask);
          targetDef->setNumPasses(1u);

          Ogre::CompositorPassDef *passDef =
              targetDef->addPass(Ogre::PASS_SCENE);
          Ogre::CompositorPassSceneDef *passScene =
              static_cast<Ogre::CompositorPassSceneDef *>(passDef);

          passScene->mShadowMapIdx = currentShadowMapIdx + i;
          passScene->mIncludeOverlays = false;
          // passScene->mVisibilityMask = Ogre::VisibilityFlags::RESERVED_VISIBILITY_FLAGS;
        }
      }
      // else
      // {
        shadowMapIdx += numSplits;
      // }
      ++itor;
    }

    // Pass scene for point lights last
    shadowMapIdx = 0;
    itor = _shadowParams.begin();
    while (itor != end)
    {
      const Ogre::ShadowNodeHelper::ShadowParam &shadowParam = *itor;
      if (shadowParam.atlasId == atlasId &&
          shadowParam.supportedLightTypes & pointMask)
      {
        // Render to cubemap, each face clear + render
        for (Ogre::uint32 i = 0; i < 6u; ++i)
        {
          Ogre::CompositorTargetDef *targetDef =
              shadowNodeDef->addTargetPass("tmpCubemap", i);
          targetDef->setNumPasses(2u);
          targetDef->setShadowMapSupportedLightTypes(
              shadowParam.supportedLightTypes & pointMask);
          {
              //Scene pass
              Ogre::CompositorPassDef *passDef = targetDef->addPass( Ogre::PASS_SCENE );
              Ogre::CompositorPassSceneDef *passScene =
                      static_cast<Ogre::CompositorPassSceneDef*>( passDef );
              passScene->setAllLoadActions( Ogre::LoadAction::Clear );
              passScene->setAllClearColours( Ogre::ColourValue( 0.0f, 0.0f, 0.0f, 0.0f ) );
              passScene->mClearDepth = 1.0f;
              passScene->mCameraCubemapReorient = true;
              passScene->mShadowMapIdx = shadowMapIdx;
              passScene->mIncludeOverlays = false;
              // passScene->mVisibilityMask = Ogre::VisibilityFlags::RESERVED_VISIBILITY_FLAGS;
          }
        }

        // Copy to the atlas using a pass quad
        // (Cubemap -> DPSM / Dual Paraboloid).
        Ogre::CompositorTargetDef *targetDef =
            shadowNodeDef->addTargetPass(texName);
        targetDef->setShadowMapSupportedLightTypes(
            shadowParam.supportedLightTypes & pointMask);
        targetDef->setNumPasses(1u);
        Ogre::CompositorPassDef *passDef = targetDef->addPass(Ogre::PASS_QUAD);
        Ogre::CompositorPassQuadDef *passQuad =
            static_cast<Ogre::CompositorPassQuadDef *>(passDef);
        passQuad->mMaterialIsHlms = false;
        passQuad->mMaterialName = "Ogre/DPSM/CubeToDpsm";
        passQuad->addQuadTextureSource(0, "tmpCubemap");
        passQuad->mShadowMapIdx = shadowMapIdx;
      }
      const size_t numSplits = shadowParam.technique ==
          Ogre::SHADOWMAP_PSSM ? shadowParam.numPssmSplits : 1u;
      shadowMapIdx += numSplits;
      ++itor;
    }
  }
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::RebuildImpl()
{
  this->RebuildTarget();
  this->RebuildMaterial();
  this->RebuildCompositor();
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetMaterial(MaterialPtr _material)
{
  this->material = _material;

  // Have to rebuild the target so there is something to apply the applicator to
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::RebuildMaterial()
{
  if (this->material)
  {
    std::cerr << "/* Ogre2RenderTargetMaterial */" << '\n';
    Ogre2Material *ogreMaterial = dynamic_cast<Ogre2Material *>(
        this->material.get());
    Ogre::MaterialPtr matPtr = ogreMaterial->Material();

    Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
    // Ogre::TextureGpu *target = this->RenderTarget();
    // // TODO(ahcorde): Review this
    this->materialApplicator.reset(new Ogre2RenderTargetMaterial(
        sceneMgr, this->ogreCamera, matPtr.get()));
  }
}

//////////////////////////////////////////////////
// Ogre2RenderTexture
//////////////////////////////////////////////////
Ogre2RenderTexture::Ogre2RenderTexture()
{
}

//////////////////////////////////////////////////
Ogre2RenderTexture::~Ogre2RenderTexture()
{
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::Destroy()
{
  this->DestroyTarget();
}

//////////////////////////////////////////////////
Ogre::TextureGpu *Ogre2RenderTexture::RenderTarget() const
{
  return this->ogreTexture;
}


//////////////////////////////////////////////////
void Ogre2RenderTexture::RebuildTarget()
{
  this->DestroyTarget();
  this->BuildTarget();
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::DestroyTarget()
{
  std::cerr << "/* DestroyTarget */" << '\n';
  if (nullptr == this->ogreTexture)
    return;

  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::CompositorManager2 *compositorManager = root->getCompositorManager2();
  compositorManager->removeAllWorkspaces();
  this->ogreCompositorWorkspace2 = 0;
  this->ogreCompositorWorkspace = 0;

  compositorManager->removeAllNodeDefinitions();
  compositorManager->removeAllShadowNodeDefinitions();
  compositorManager->removeAllWorkspaceDefinitions();
  // compositorManager->removeAllWorkspaces();


  // compositorManager->removeWorkspace( this->ogreCompositorWorkspace2 );
  // this->ogreCompositorWorkspace2 = 0;
  // compositorManager->removeWorkspace( this->ogreCompositorWorkspace );
  // this->ogreCompositorWorkspace = 0;
  //
  // compositorManager->removeWorkspace( this->ogreCompositorWorkspaceDefault );
  // this->ogreCompositorWorkspaceDefault = 0;
  //
  //
  // std::string shadowNodeDefName = "PbsMaterialsShadowNode";
  // if (compositorManager->hasShadowNodeDefinition(shadowNodeDefName))
  //   compositorManager->removeShadowNodeDefinition(shadowNodeDefName);


  // Ogre::TextureGpuManager *textureManager = root->getRenderSystem()->getTextureGpuManager();
  // textureManager->destroyTexture(this->ogreTexture);
  // textureManager->unload(this->ogreTexture->getName());
  // textureManager->remove(this->ogreTexture->getName());

  // auto &manager = Ogre::TextureManager::getSingleton();
  // manager.unload(this->ogreTexture->getName());
  // manager.remove(this->ogreTexture->getName());

  // TODO(anyone) there is memory leak when a render texture is destroyed.
  // The RenderSystem::_cleanupDepthBuffers method used in ogre1 does not
  // seem to work in ogre2

  // this->ogreTexture = nullptr;
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::BuildTarget()
{
  // check if target fsaa is supported
  unsigned int fsaa = 0;
  std::vector<unsigned int> fsaaLevels =
      Ogre2RenderEngine::Instance()->FSAALevels();
  unsigned int targetFSAA = this->antiAliasing;
  auto const it = std::find(fsaaLevels.begin(), fsaaLevels.end(), targetFSAA);
  if (it != fsaaLevels.end())
  {
    fsaa = targetFSAA;
  }
  else
  {
    // output warning but only do it once
    static bool ogre2FSAAWarn = false;
    if (ogre2FSAAWarn)
    {
      ignwarn << "Anti-aliasing level of '" << this->antiAliasing << "' "
              << "is not supported. Setting to 0" << std::endl;
      ogre2FSAAWarn = true;
    }
  }

  auto engine = Ogre2RenderEngine::Instance();
  this->ogreTexture = engine->getWindow()->getTexture();
  // auto engine = Ogre2RenderEngine::Instance();
  // auto ogreRoot = engine->OgreRoot();
  //
  // Ogre::TextureGpuManager *textureMgr = ogreRoot->getRenderSystem()->getTextureGpuManager();
  // Ogre::TextureGpu *texture = textureMgr->createOrRetrieveTexture(
  //                                 "General",
  //                                 Ogre::GpuPageOutStrategy::Discard,
  //                                 Ogre::TextureFlags::ManualTexture,
  //                                 Ogre::TextureTypes::Type2D,
  //                                 Ogre::BLANKSTRING,
  //                                 0u );
  // texture->setPixelFormat( Ogre::PFG_RGBA8_UNORM_SRGB );
  // texture->setTextureType( Ogre::TextureTypes::Type2D );
  // texture->setNumMipmaps( 1u );
  // texture->setResolution(width, height);
}

//////////////////////////////////////////////////
unsigned int Ogre2RenderTexture::GLId() const
{
  if (!this->ogreTexture)
    return 0;

  unsigned int texId;
  this->ogreTexture->getCustomAttribute("GLID", &texId);
  return static_cast<unsigned int>(texId);
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::PreRender()
{
  Ogre2RenderTarget::PreRender();
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::PostRender()
{
  Ogre2RenderTarget::PostRender();
}

//////////////////////////////////////////////////
// Ogre2RenderWindow
//////////////////////////////////////////////////
Ogre2RenderWindow::Ogre2RenderWindow()
{
}

//////////////////////////////////////////////////
Ogre2RenderWindow::~Ogre2RenderWindow()
{
}

//////////////////////////////////////////////////
Ogre::TextureGpu *Ogre2RenderWindow::RenderTarget() const
{
  return this->ogreRenderWindow;
}

//////////////////////////////////////////////////
void Ogre2RenderWindow::Destroy()
{
  // TODO(anyone)
}

//////////////////////////////////////////////////
void Ogre2RenderWindow::RebuildTarget()
{
  std::cerr << "/*  Ogre2RenderWindow::RebuildTarget() */" << '\n';
  // TODO(anyone): determine when to rebuild
  // ie. only when ratio or handle changes!
  // e.g. sizeDirty?
  if (!this->ogreRenderWindow)
    this->BuildTarget();

  Ogre::Window *window =
      dynamic_cast<Ogre::Window *>(this->ogreRenderWindow);
  window->requestResolution(this->width, this->height);
  window->windowMovedOrResized();
}

//////////////////////////////////////////////////
void Ogre2RenderWindow::BuildTarget()
{
  std::cerr << "void Ogre2RenderWindow::BuildTarget()" << '\n';
  auto engine = Ogre2RenderEngine::Instance();
  engine->CreateRenderWindow(this->handle,
      this->width,
      this->height,
      this->ratio,
      this->antiAliasing);

  this->ogreRenderWindow = engine->getWindow()->getTexture();
}
