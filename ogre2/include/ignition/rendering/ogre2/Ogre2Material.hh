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
#ifndef IGNITION_RENDERING_OGRE2_OGRE2MATERIAL_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2MATERIAL_HH_

#include <memory>
#include <string>

#include "ignition/rendering/base/BaseMaterial.hh"
#include "ignition/rendering/ogre2/Ogre2Object.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class Ogre2MaterialPrivate;

    /// \brief Ogre 2.x implementation of the material class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Material :
      public BaseMaterial<Ogre2Object>
    {
      /// \brief Constructor
      protected: Ogre2Material();

      /// \brief Destructor
      public: virtual ~Ogre2Material();

      // Documentation inherited
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual math::Color Diffuse() const override;

      // Documentation inherited
      public: virtual void SetDiffuse(const math::Color &_color) override;

      // Documentation inherited
      public: virtual math::Color Specular() const override;

      // Documentation inherited
      public: virtual void SetSpecular(const math::Color &_color) override;

      // Documentation inherited
      public: virtual math::Color Emissive() const override;

      // Documentation inherited
      public: virtual void SetEmissive(const math::Color &_color) override;

      // Documentation inherited
      public: virtual void SetTransparency(const double _transparency) override;

      // Documentation inherited
      public: virtual void SetAlphaFromTexture(bool _enabled,
          double _alpha = 0.5, bool _twoSided = true) override;

      // Documentation inherited
      public: virtual float RenderOrder() const override;

      // Documentation inherited
      // Review the official documentation to get more details about this
      // parameter, in particular mDepthBiasConstant
      public: virtual void SetRenderOrder(const float _renderOrder) override;

      // Documentation inherited
      public: virtual bool ReceiveShadows() const override;

      // Documentation inherited
      public: virtual void SetReceiveShadows(const bool _receiveShadows)
          override;

      // Documentation inherited
      public: virtual bool HasTexture() const override;

      // Documentation inherited
      public: virtual std::string Texture() const override;

      // Documentation inherited
      public: virtual void SetTexture(const std::string &_name) override;

      // Documentation inherited
      public: virtual void ClearTexture() override;

      // Documentation inherited
      public: virtual bool HasNormalMap() const override;

      // Documentation inherited
      public: virtual std::string NormalMap() const override;

      // Documentation inherited
      public: virtual void SetNormalMap(const std::string &_name) override;

      // Documentation inherited
      public: virtual void ClearNormalMap() override;

      // Documentation inherited
      public: virtual bool HasRoughnessMap() const override;

      // Documentation inherited
      public: virtual std::string RoughnessMap() const override;

      // Documentation inherited
      public: virtual void SetRoughnessMap(const std::string &_name) override;

      // Documentation inherited
      public: virtual void ClearRoughnessMap() override;

      // Documentation inherited
      public: virtual bool HasMetalnessMap() const override;

      // Documentation inherited
      public: virtual std::string MetalnessMap() const override;

      // Documentation inherited
      public: virtual void SetMetalnessMap(const std::string &_name) override;

      // Documentation inherited
      public: virtual void ClearMetalnessMap() override;

      // Documentation inherited
      public: virtual bool HasEnvironmentMap() const override;

      // Documentation inherited
      public: virtual std::string EnvironmentMap() const override;

      // Documentation inherited
      public: virtual void SetEnvironmentMap(const std::string &_name) override;

      // Documentation inherited
      public: virtual void ClearEnvironmentMap() override;

      // Documentation inherited
      public: virtual void SetRoughness(const float _roughness) override;

      // Documentation inherited
      public: virtual bool HasEmissiveMap() const override;

      // Documentation inherited
      public: virtual std::string EmissiveMap() const override;

      // Documentation inherited
      public: virtual void SetEmissiveMap(const std::string &_name) override;

      // Documentation inherited
      public: virtual void ClearEmissiveMap() override;

      // Documentation inherited
      public: virtual bool HasLightMap() const override;

      // Documentation inherited
      public: virtual std::string LightMap() const override;

      // Documentation inherited
      public: virtual unsigned int LightMapTexCoordSet() const override;

      // Documentation inherited
      public: virtual void SetLightMap(const std::string &_name,
          unsigned int _uvSet = 0u) override;

      // Documentation inherited
      public: virtual void ClearLightMap() override;

      // Documentation inherited
      public: virtual float Roughness() const override;

      // Documentation inherited
      public: virtual void SetMetalness(const float _roughness) override;

      // Documentation inherited
      public: virtual float Metalness() const override;

      /// \brief Return ogre low level material
      /// \return Ogre material pointer
      public: virtual Ogre::MaterialPtr Material();

      /// \brief Return ogre Hlms material pbs datablock
      /// \return Ogre Hlms pbs datablock
      public: virtual Ogre::HlmsPbsDatablock *Datablock() const;

      /// \brief Return ogre Hlms material unlit datablock
      /// \return Ogre Hlms unlit datablock
      public: virtual Ogre::HlmsUnlitDatablock *UnlitDatablock();

      /// \brief Fill the input unlit datablock with current material
      /// properties from the pbs datablock
      /// \param[in] _datablock Unlit datablock to fill
      public: virtual void FillUnlitDatablock(
          Ogre::HlmsUnlitDatablock *_datablock) const;

      // Documentation inherited.
      // \sa BaseMaterial::PreRender()
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual enum MaterialType Type() const override;

      // Documentation inherited
      public: virtual bool DepthCheckEnabled() const override;

      // Documentation inherited
      public: virtual void SetDepthCheckEnabled(bool _enabled) override;

      // Documentation inherited
      public: virtual bool DepthWriteEnabled() const override;

      // Documentation inherited
      public: virtual void SetDepthWriteEnabled(bool _enabled) override;

      /// \brief Set the texture map for this material
      /// \param[in] _texture Name of the texture.
      /// \param[in] _type Type of texture, i.e. diffuse, normal, roughness,
      /// metalness
      protected: virtual void SetTextureMapImpl(const std::string &_texture,
          Ogre::PbsTextureTypes _type);

      /// \brief Get a pointer to the ogre texture by name
      /// \return Ogre texture
      protected: virtual Ogre::TextureGpu *Texture(const std::string &_name);

      /// \brief Updates the material transparency in the engine,
      /// based on transparency and diffuse alpha values
      protected: virtual void UpdateTransparency();

      // Documentation inherited.
      protected: virtual void Init() override;

      /// \brief  Ogre material. Mainly used for render targets.
      protected: Ogre::MaterialPtr ogreMaterial;

      /// \brief  Ogre data block containing all pbs material properties
      protected: Ogre::HlmsPbsDatablock *ogreDatablock = nullptr;

      /// \brief  Ogre data block containing all unlit material properties
      protected: Ogre::HlmsUnlitDatablock *ogreUnlitDatablock = nullptr;

      /// \brief  Ogre high level physical based shading object
      protected: Ogre::HlmsPbs *ogreHlmsPbs = nullptr;

      /// \brief Name of the texture
      protected: std::string textureName;

      /// \brief Name of the normal map
      protected: std::string normalMapName;

      /// \brief Name of the roughness map
      protected: std::string roughnessMapName;

      /// \brief Name of the metalness map
      protected: std::string metalnessMapName;

      /// \brief Name of the environment map
      protected: std::string environmentMapName;

      /// \brief Name of the emissive map
      protected: std::string emissiveMapName;

      /// \brief Name of the light map
      protected: std::string lightMapName;

      /// \brief Texture coorindate set used by the light map
      protected: unsigned int lightMapUvSet = 0u;

      /// \brief Unique id assigned to ogre hlms datablock
      protected: std::string ogreDatablockId;

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2MaterialPrivate> dataPtr;

      /// \brief Only an ogre scene can create an ogre material
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
