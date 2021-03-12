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

// Note this include is placed in the src file because
// otherwise ogre produces compile errors
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <Hlms/Pbs/OgreHlmsPbsDatablock.h>
#include <Hlms/Unlit/OgreHlmsUnlitDatablock.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>

#include "ignition/rendering/ShaderParams.hh"
#include "ignition/rendering/ShaderType.hh"
#include "ignition/rendering/ogre2/Ogre2Material.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"

/// \brief Private data for the Ogre2Material class
class ignition::rendering::Ogre2MaterialPrivate
{
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2Material::Ogre2Material()
  : dataPtr(std::make_unique<Ogre2MaterialPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2Material::~Ogre2Material()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2Material::Destroy()
{
  if (!this->Scene()->IsInitialized())
    return;

  if (!this->ogreDatablock)
    return;

  this->ogreHlmsPbs->destroyDatablock(this->ogreDatablockId);
  this->ogreDatablock = nullptr;

  if (this->ogreUnlitDatablock)
  {
    this->ogreUnlitDatablock->getCreator()->destroyDatablock(
        this->ogreUnlitDatablock->getName());
    this->ogreUnlitDatablock = nullptr;
  }

  // remove low level ogre material used by render targets
  if (this->ogreMaterial)
  {
    Ogre::MaterialManager &matManager = Ogre::MaterialManager::getSingleton();
    matManager.remove(this->ogreMaterial);
    this->ogreMaterial.reset();
  }
}

//////////////////////////////////////////////////
math::Color Ogre2Material::Diffuse() const
{
  Ogre::Vector3 color =
      this->ogreDatablock->getDiffuse();
  return math::Color(color.x, color.y, color.z, this->diffuse.A());
}

//////////////////////////////////////////////////
void Ogre2Material::SetDiffuse(const math::Color &_color)
{
  BaseMaterial::SetDiffuse(_color);
  this->ogreDatablock->setDiffuse(
      Ogre::Vector3(_color.R(), _color.G(), _color.B()));
  this->UpdateTransparency();
}

//////////////////////////////////////////////////
math::Color Ogre2Material::Specular() const
{
  Ogre::Vector3 color =
      this->ogreDatablock->getSpecular();
  return math::Color(color.x, color.y, color.z, 1.0);
}

//////////////////////////////////////////////////
void Ogre2Material::SetSpecular(const math::Color &_color)
{
  this->ogreDatablock->setSpecular(
      Ogre::Vector3(_color.R(), _color.G(), _color.B()));
}

//////////////////////////////////////////////////
math::Color Ogre2Material::Emissive() const
{
  Ogre::Vector3 color =
      this->ogreDatablock->getEmissive();
  return math::Color(color.x, color.y, color.z, 1.0);
}

//////////////////////////////////////////////////
void Ogre2Material::SetEmissive(const math::Color &_color)
{
  this->ogreDatablock->setEmissive(
      Ogre::Vector3(_color.R(), _color.G(), _color.B()));
}

//////////////////////////////////////////////////
void Ogre2Material::SetTransparency(const double _transparency)
{
  this->transparency = std::min(std::max(_transparency, 0.0), 1.0);
  this->UpdateTransparency();
}

//////////////////////////////////////////////////
void Ogre2Material::UpdateTransparency()
{
  Ogre::HlmsPbsDatablock::TransparencyModes mode;
  double opacity = (1.0 - this->transparency) * this->diffuse.A();
  if (math::equal(opacity, 1.0))
    mode = Ogre::HlmsPbsDatablock::None;
  else
    mode = Ogre::HlmsPbsDatablock::Transparent;

  // from ogre documentation: 0 = full transparency and 1 = fully opaque
  this->ogreDatablock->setTransparency(opacity, mode);
}

//////////////////////////////////////////////////
void Ogre2Material::SetAlphaFromTexture(bool _enabled,
    double _alpha, bool _twoSided)
{
  BaseMaterial::SetAlphaFromTexture(_enabled, _alpha, _twoSided);
  if (_enabled)
  {
    this->ogreDatablock->setAlphaTest(Ogre::CMPF_GREATER_EQUAL);
    Ogre::HlmsBlendblock block;
    block.setBlendType(Ogre::SBT_TRANSPARENT_ALPHA);
    this->ogreDatablock->setBlendblock(block);
  }
  else
  {
    this->ogreDatablock->setAlphaTest(Ogre::CMPF_ALWAYS_PASS);
  }
  this->ogreDatablock->setAlphaTestThreshold(_alpha);
  this->ogreDatablock->setTwoSidedLighting(_twoSided);
}

//////////////////////////////////////////////////
float Ogre2Material::RenderOrder() const
{
  Ogre::HlmsMacroblock macroblock(
      *this->ogreDatablock->getMacroblock());
  return macroblock.mDepthBiasConstant;
}

//////////////////////////////////////////////////
void Ogre2Material::SetRenderOrder(const float _renderOrder)
{
  this->renderOrder = _renderOrder;
  Ogre::HlmsMacroblock macroblock(
      *this->ogreDatablock->getMacroblock());
  macroblock.mDepthBiasConstant = _renderOrder;
  this->ogreDatablock->setMacroblock(macroblock);
}

//////////////////////////////////////////////////
bool Ogre2Material::ReceiveShadows() const
{
  return this->ogreDatablock->getReceiveShadows();
}

//////////////////////////////////////////////////
void Ogre2Material::SetReceiveShadows(const bool _receiveShadows)
{
  this->ogreDatablock->setReceiveShadows(_receiveShadows);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasTexture() const
{
  return !this->textureName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::Texture() const
{
  return this->textureName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetTexture(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearTexture();
    return;
  }

  this->textureName = _name;
  this->SetTextureMapImpl(this->textureName, Ogre::PBSM_DIFFUSE);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearTexture()
{
  this->textureName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_DIFFUSE, this->textureName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasNormalMap() const
{
  return !this->normalMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::NormalMap() const
{
  return this->normalMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetNormalMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearNormalMap();
    return;
  }

  this->normalMapName = _name;
  this->SetTextureMapImpl(this->normalMapName, Ogre::PBSM_NORMAL);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearNormalMap()
{
  this->normalMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_NORMAL, this->normalMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasRoughnessMap() const
{
  return !this->roughnessMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::RoughnessMap() const
{
  return this->roughnessMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetRoughnessMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearRoughnessMap();
    return;
  }

  this->roughnessMapName = _name;
  this->SetTextureMapImpl(this->roughnessMapName, Ogre::PBSM_ROUGHNESS);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearRoughnessMap()
{
  this->roughnessMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_ROUGHNESS, this->roughnessMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasMetalnessMap() const
{
  return !this->metalnessMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::MetalnessMap() const
{
  return this->metalnessMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetMetalnessMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearMetalnessMap();
    return;
  }

  this->metalnessMapName = _name;
  this->SetTextureMapImpl(this->metalnessMapName, Ogre::PBSM_METALLIC);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearMetalnessMap()
{
  this->metalnessMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_METALLIC, this->metalnessMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasEnvironmentMap() const
{
  return !this->environmentMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::EnvironmentMap() const
{
  return this->environmentMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetEnvironmentMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearEnvironmentMap();
    return;
  }

  this->environmentMapName = _name;
  this->SetTextureMapImpl(this->environmentMapName, Ogre::PBSM_REFLECTION);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearEnvironmentMap()
{
  this->environmentMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_REFLECTION, this->environmentMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasEmissiveMap() const
{
  return !this->emissiveMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::EmissiveMap() const
{
  return this->emissiveMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetEmissiveMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearEmissiveMap();
    return;
  }

  this->emissiveMapName = _name;
  this->SetTextureMapImpl(this->emissiveMapName, Ogre::PBSM_EMISSIVE);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearEmissiveMap()
{
  this->emissiveMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_EMISSIVE, this->emissiveMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasLightMap() const
{
  return !this->lightMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::LightMap() const
{
  return this->lightMapName;
}

//////////////////////////////////////////////////
unsigned int Ogre2Material::LightMapTexCoordSet() const
{
  return this->lightMapUvSet;
}

//////////////////////////////////////////////////
void Ogre2Material::SetLightMap(const std::string &_name, unsigned int _uvSet)
{
  if (_name.empty())
  {
    this->ClearLightMap();
    return;
  }

  this->lightMapName = _name;
  this->lightMapUvSet = _uvSet;

  // reserve detail map 0 for light map
  Ogre::PbsTextureTypes type = Ogre::PBSM_DETAIL0;

  // lightmap usually uses a different tex coord set
  this->SetTextureMapImpl(this->lightMapName, type);

  this->ogreDatablock->setTextureUvSource(type, this->lightMapUvSet);

  // PBSM_BLEND_OVERLAY and PBSM_BLEND_MULTIPLY2X produces better results
  this->ogreDatablock->setDetailMapBlendMode(0, Ogre::PBSM_BLEND_OVERLAY);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearLightMap()
{
  this->lightMapName = "";
  this->lightMapUvSet = 0u;
  this->ogreDatablock->setTexture(Ogre::PBSM_DETAIL0, this->lightMapName);
}

//////////////////////////////////////////////////
void Ogre2Material::SetRoughness(const float _roughness)
{
  this->ogreDatablock->setRoughness(_roughness);
}

//////////////////////////////////////////////////
float Ogre2Material::Roughness() const
{
  return this->ogreDatablock->getRoughness();
}

//////////////////////////////////////////////////
void Ogre2Material::SetMetalness(const float _metalness)
{
  this->ogreDatablock->setMetalness(_metalness);
}

//////////////////////////////////////////////////
float Ogre2Material::Metalness() const
{
  return this->ogreDatablock->getMetalness();
}

//////////////////////////////////////////////////
void Ogre2Material::PreRender()
{
}

//////////////////////////////////////////////////
Ogre::MaterialPtr Ogre2Material::Material()
{
  if (!this->ogreMaterial)
  {
    // low level ogre material used by render targets
    Ogre::MaterialManager &matManager = Ogre::MaterialManager::getSingleton();
    this->ogreMaterial = matManager.create(this->name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }

  return this->ogreMaterial;
}

//////////////////////////////////////////////////
Ogre::HlmsPbsDatablock *Ogre2Material::Datablock() const
{
  return this->ogreDatablock;
}

//////////////////////////////////////////////////
void Ogre2Material::SetTextureMapImpl(const std::string &_texture,
  Ogre::PbsTextureTypes _type)
{
  // FIXME(anyone) need to keep baseName = _texture for all meshes. Refer to
  // https://github.com/ignitionrobotics/ign-rendering/issues/139
  // for more details
  std::string baseName = _texture;
  if (common::isFile(_texture))
  {
    baseName = common::basename(_texture);
    size_t idx = _texture.rfind(baseName);
    if (idx != std::string::npos)
    {
      std::string dirPath = _texture.substr(0, idx);
      if (!dirPath.empty() &&
          !Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(
          dirPath))
      {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            dirPath, "FileSystem", "General");
      }
    }
  }

  // temp workaround check if the model is a OBJ file
  {
    size_t idx = _texture.rfind("meshes");
    if (idx != std::string::npos)
    {
      std::string objFile =
        common::joinPaths(_texture.substr(0, idx), "meshes", "model.obj");
      if (common::isFile(objFile))
        baseName = _texture;
    }
  }

  // Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  // Ogre::TextureGpuManager *textureMgr =
  //   root->getRenderSystem()->getTextureGpuManager();
  //
  // Ogre::TextureGpu *texture = textureMgr->createOrRetrieveTexture(
  //   baseName,
  //   Ogre::GpuPageOutStrategy::Discard,
  //   Ogre::TextureFlags::ManualTexture,
  //   Ogre::TextureTypes::Type2D,
  //   Ogre::BLANKSTRING,
  //   0u);

  // Ogre::HlmsSamplerblock samplerblock( *datablock->getSamplerblock( Ogre::PBSM_ROUGHNESS ) );
  Ogre::HlmsSamplerblock samplerBlockRef;
  samplerBlockRef.mU = Ogre::TAM_WRAP;
  samplerBlockRef.mV = Ogre::TAM_WRAP;
  samplerBlockRef.mW = Ogre::TAM_WRAP;
  //
  // this->ogreDatablock->setTexture(_type, texture,
  //     &samplerBlockRef);
  this->ogreDatablock->setTexture(_type, baseName, &samplerBlockRef);

  // disable alpha from texture if texture does not have an alpha channel
  // otherwise this becomes a transparent material
  // if (_type == Ogre::PBSM_DIFFUSE)
  // {
  //   if (this->TextureAlphaEnabled() && !texture->hasAlpha())
  //   {
  //     this->SetAlphaFromTexture(false, this->AlphaThreshold(),
  //         this->TwoSidedEnabled());
  //   }
  // }
}

//////////////////////////////////////////////////
Ogre::TextureGpu* Ogre2Material::Texture(const std::string &_name)
{
  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::TextureGpuManager *textureMgr =
    root->getRenderSystem()->getTextureGpuManager();
  std::cerr << "Texture " << _name << '\n';
  Ogre::TextureGpu *texture = textureMgr->createOrRetrieveTexture(_name,
        Ogre::GpuPageOutStrategy::Discard,
        Ogre::TextureFlags::ManualTexture,
        Ogre::TextureTypes::Type2D);

  return texture;
}

//////////////////////////////////////////////////
void Ogre2Material::Init()
{
  BaseMaterial::Init();

  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::HlmsManager *hlmsManager = root->getHlmsManager();
  this->ogreHlmsPbs = static_cast<Ogre::HlmsPbs*>(
      hlmsManager->getHlms(Ogre::HLMS_PBS));

  if (!this->ogreHlmsPbs)
  {
    ignerr << "Ogre HLMS PBS not ready. Is Ogre2 Render Engine initiallized?"
           << std::endl;
    return;
  }
  this->ogreDatablockId = this->Scene()->Name() + "::" + this->name;
  this->ogreDatablock = static_cast<Ogre::HlmsPbsDatablock *>(
      this->ogreHlmsPbs->createDatablock(
      this->ogreDatablockId, this->name,
      Ogre::HlmsMacroblock(), Ogre::HlmsBlendblock(), Ogre::HlmsParamVec()));

  // use metal workflow as default
  this->ogreDatablock->setWorkflow(Ogre::HlmsPbsDatablock::MetallicWorkflow);

  this->Reset();
}

//////////////////////////////////////////////////
enum MaterialType Ogre2Material::Type() const
{
  return MaterialType::MT_PBS;
}

//////////////////////////////////////////////////
bool Ogre2Material::DepthCheckEnabled() const
{
  const Ogre::HlmsMacroblock *macroblock = this->ogreDatablock->getMacroblock();
  return macroblock->mDepthCheck;
}

//////////////////////////////////////////////////
void Ogre2Material::SetDepthCheckEnabled(bool _enabled)
{
  Ogre::HlmsMacroblock macroblock(
      *this->ogreDatablock->getMacroblock());
  macroblock.mDepthCheck = _enabled;
  this->ogreDatablock->setMacroblock(macroblock);
}

//////////////////////////////////////////////////
bool Ogre2Material::DepthWriteEnabled() const
{
  const Ogre::HlmsMacroblock *macroblock = this->ogreDatablock->getMacroblock();
  return macroblock->mDepthWrite;
}

//////////////////////////////////////////////////
void Ogre2Material::SetDepthWriteEnabled(bool _enabled)
{
  Ogre::HlmsMacroblock macroblock(
      *this->ogreDatablock->getMacroblock());
  macroblock.mDepthWrite = _enabled;
  this->ogreDatablock->setMacroblock(macroblock);
}

//////////////////////////////////////////////////
Ogre::HlmsUnlitDatablock *Ogre2Material::UnlitDatablock()
{
  if (!this->ogreUnlitDatablock)
  {
    // Create the internal UNLIT material.
    Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
    Ogre::HlmsManager *hlmsManager = root->getHlmsManager();
    Ogre::HlmsUnlit *ogreHlmsUnlit = static_cast<Ogre::HlmsUnlit *>(
        hlmsManager->getHlms(Ogre::HLMS_UNLIT));
    if (!ogreHlmsUnlit)
    {
      ignerr << "Ogre HLMS UNLIT not ready. Is Ogre2 Render Engine "
             << "initiallized?" << std::endl;
      return nullptr;
    }

    std::string unlitName = this->name + "::Unlit";
    this->ogreUnlitDatablock =
        static_cast<Ogre::HlmsUnlitDatablock *>(
        ogreHlmsUnlit->createDatablock(
        unlitName, unlitName,
        Ogre::HlmsMacroblock(), Ogre::HlmsBlendblock(), Ogre::HlmsParamVec()));
  }

  this->FillUnlitDatablock(this->ogreUnlitDatablock);
  return this->ogreUnlitDatablock;
}

//////////////////////////////////////////////////
void Ogre2Material::FillUnlitDatablock(Ogre::HlmsUnlitDatablock *_datablock)
    const
{
  if (!this->textureName.empty())
  {
    std::string baseName = common::basename(this->textureName);

    Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
    Ogre::TextureGpuManager *textureMgr =
      root->getRenderSystem()->getTextureGpuManager();
    Ogre::TextureGpu *texture = textureMgr->createOrRetrieveTexture(baseName,
          Ogre::GpuPageOutStrategy::Discard,
          Ogre::TextureFlags::ManualTexture,
          Ogre::TextureTypes::Type2D);

    _datablock->setTexture(0, texture);
  }

  auto samplerblock = this->ogreDatablock->getSamplerblock(Ogre::PBSM_DIFFUSE);
  if (samplerblock)
    _datablock->setSamplerblock(0, *samplerblock);
  _datablock->setMacroblock(this->ogreDatablock->getMacroblock());
  _datablock->setBlendblock(this->ogreDatablock->getBlendblock());

  _datablock->setUseColour(true);
  Ogre::Vector3 c = this->ogreDatablock->getDiffuse();
  _datablock->setColour(Ogre::ColourValue(c.x, c.y, c.z));
}
