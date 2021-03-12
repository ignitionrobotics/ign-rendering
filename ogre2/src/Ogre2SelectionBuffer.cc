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

#include <memory>
#include <ignition/math/Color.hh>

#include "ignition/common/Console.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2MaterialSwitcher.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2SelectionBuffer.hh"

using namespace ignition;
using namespace rendering;

class ignition::rendering::Ogre2SelectionBufferPrivate
{
  /// \brief This is a material listener and a RenderTargetListener.
  /// The material switcher is applied to only the selection camera
  /// and not applied globally to all targets. The class associates a
  /// color to an ogre entity
  public: std::unique_ptr<Ogre2MaterialSwitcher> materialSwitcher;

  /// \brief Ogre scene manager
  public: Ogre2ScenePtr scene;

  /// \brief Ogre scene manager
  public: Ogre::SceneManager *sceneMgr = nullptr;

  /// \brief Pointer to the camera that will be used as the reference
  /// for selection
  public: Ogre::Camera *camera = nullptr;

  /// \brief Selection buffer's render to texture camera
  public: Ogre::Camera *selectionCamera  = nullptr;

  /// \brief Ogre render texture
  public: Ogre::TextureGpu *renderTexture = nullptr;

  /// \brief Ogre's compositor workspace - the main interface to render
  /// into a render target or render texture.
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace = nullptr;
};

/////////////////////////////////////////////////
Ogre2SelectionBuffer::Ogre2SelectionBuffer(const std::string &_cameraName,
    Ogre2ScenePtr _scene): dataPtr(new Ogre2SelectionBufferPrivate)
{
  this->dataPtr->scene = _scene;
  this->dataPtr->sceneMgr = _scene->OgreSceneManager();

  this->dataPtr->camera = this->dataPtr->sceneMgr->findCameraNoThrow(
      _cameraName);
  if (!this->dataPtr->camera)
  {
    ignerr << "No camera found. Unable to create Ogre 2 selection buffer "
           << std::endl;
    return;
  }

  this->dataPtr->selectionCamera =
      this->dataPtr->sceneMgr->createCamera(_cameraName + "_selection_buffer");

  this->dataPtr->selectionCamera->setFOVy(this->dataPtr->camera->getFOVy());
  this->dataPtr->selectionCamera->setNearClipDistance(this->dataPtr->camera->getNearClipDistance());
  this->dataPtr->selectionCamera->setFarClipDistance(this->dataPtr->camera->getFarClipDistance());
  this->dataPtr->selectionCamera->detachFromParent();
  this->dataPtr->sceneMgr->getRootSceneNode()->attachObject(this->dataPtr->selectionCamera);

  this->dataPtr->materialSwitcher.reset(
      new Ogre2MaterialSwitcher(this->dataPtr->scene));
  this->CreateRTTBuffer();
}

/////////////////////////////////////////////////
Ogre2SelectionBuffer::~Ogre2SelectionBuffer()
{
  this->DeleteRTTBuffer();

  // remove selection buffer camera
  this->dataPtr->sceneMgr->destroyCamera(this->dataPtr->selectionCamera);
}

/////////////////////////////////////////////////
void Ogre2SelectionBuffer::Update()
{
  if (!this->dataPtr->renderTexture)
    return;

  this->dataPtr->materialSwitcher->Reset();

  // manual update
  this->dataPtr->ogreCompositorWorkspace->setEnabled(true);
  auto engine = Ogre2RenderEngine::Instance();
  engine->OgreRoot()->renderOneFrame();
  this->dataPtr->ogreCompositorWorkspace->setEnabled(false);
}

/////////////////////////////////////////////////
void Ogre2SelectionBuffer::DeleteRTTBuffer()
{
  // TODO (ahcorde)
  // auto &manager = Ogre::TextureManager::getSingleton();
  // manager.unload(this->dataPtr->renderTexture->getName());
  // manager.remove(this->dataPtr->renderTexture->getName());
  std::cerr << "DeleteRTT" << '\n';
}

/////////////////////////////////////////////////
void Ogre2SelectionBuffer::CreateRTTBuffer()
{
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();

  Ogre::TextureGpuManager *textureMgr = ogreRoot->getRenderSystem()->getTextureGpuManager();
  this->dataPtr->renderTexture =
      textureMgr->createOrRetrieveTexture("SelectionPassTex",
                                          Ogre::GpuPageOutStrategy::SaveToSystemRam,
                                          Ogre::TextureFlags::RenderToTexture,
                                          Ogre::TextureTypes::Type2D );
  this->dataPtr->renderTexture->setResolution(800, 600);
  this->dataPtr->renderTexture->setNumMipmaps(1u);
  this->dataPtr->renderTexture->setPixelFormat(Ogre::PFG_RGBA8_UNORM_SRGB);

  this->dataPtr->renderTexture->scheduleTransitionTo(Ogre::GpuResidency::Resident);

  this->dataPtr->selectionCamera->addListener(
      this->dataPtr->materialSwitcher.get());

  // create compositor workspace for rendering
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  //Setup the cubemap's compositor.
  const Ogre::String workspaceName = "SelectionBufferWorkspace" +
      this->dataPtr->camera->getName();
  ogreCompMgr->createBasicWorkspaceDef(workspaceName,
      Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f));
  this->dataPtr->ogreCompositorWorkspace =
      ogreCompMgr->addWorkspace(this->dataPtr->scene->OgreSceneManager(),
      this->dataPtr->renderTexture,
      this->dataPtr->selectionCamera,
      workspaceName,
      false);

  // TODO(ahcorde): Review
  // set visibility mask to see only items that are selectable
  auto nodeSeq = this->dataPtr->ogreCompositorWorkspace->getNodeSequence();
  auto pass = nodeSeq[0]->_getPasses()[0]->getDefinition();
  auto scenePass = dynamic_cast<const Ogre::CompositorPassSceneDef *>(pass);
  const_cast<Ogre::CompositorPassSceneDef *>(scenePass)->mVisibilityMask =
      IGN_VISIBILITY_SELECTABLE;
}

/////////////////////////////////////////////////
Ogre::Item *Ogre2SelectionBuffer::OnSelectionClick(const int _x, const int _y)
{
  if (!this->dataPtr->renderTexture)
    return nullptr;

  if (!this->dataPtr->camera)
    return nullptr;

  Ogre::Viewport *vp = this->dataPtr->camera->getLastViewport();

  if (!vp)
    return nullptr;

  // TODO(ahcorde)
  const unsigned int targetWidth = 800;//rt->getWidth();
  const unsigned int targetHeight = 600;//rt->getHeight();

  if (_x < 0 || _y < 0 || _x >= static_cast<int>(targetWidth)
      || _y >= static_cast<int>(targetHeight))
    return nullptr;

  this->dataPtr->selectionCamera->setCustomProjectionMatrix(true,
      this->dataPtr->camera->getProjectionMatrix());
  this->dataPtr->selectionCamera->setPosition(
      this->dataPtr->camera->getDerivedPosition());
  this->dataPtr->selectionCamera->setOrientation(
      this->dataPtr->camera->getDerivedOrientation());

  // update render texture
  this->Update();

  Ogre::Image2 image;
  image.convertFromTexture(this->dataPtr->renderTexture, 0, 0);

  Ogre::ColourValue colorValue = image.getColourAt(_x, _y, 0, 0);
  ignition::math::Color cv(
    colorValue.r,
    colorValue.g,
    colorValue.b);

  cv.A(1.0);
  const std::string &entName = this->dataPtr->materialSwitcher->EntityName(cv);
  if (entName.empty())
  {
    return 0;
  }
  else
  {
    auto collection = this->dataPtr->sceneMgr->findMovableObjects(
        Ogre::ItemFactory::FACTORY_TYPE_NAME, entName);
    if (collection.empty())
      return nullptr;
    else
      return dynamic_cast<Ogre::Item *>(collection[0]);
  }
}
