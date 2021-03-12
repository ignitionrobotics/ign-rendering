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
#ifndef IGNITION_RENDERING_OGRE2_OGRE2SCENE_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2SCENE_HH_

#include <memory>
#include <string>

#include "ignition/rendering/Storage.hh"
#include "ignition/rendering/base/BaseScene.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"

#include "ignition/rendering/ogre2/Export.hh"

#include "Compositor/OgreCompositorManager2.h"


namespace Ogre
{
  class Root;
  class SceneManager;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class Ogre2ScenePrivate;
    //
    /// \brief Ogre2.x implementation of the scene class
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2Scene :
      public BaseScene
    {
      /// \brief Constructor
      /// \param[in] _id Unique scene Id
      /// \param[in] _name Scene name
      protected: Ogre2Scene(unsigned int _id, const std::string &_name);

      /// \brief Destructor
      public: virtual ~Ogre2Scene();

      // Documentation inherited.
      public: virtual void Fini() override;

      // Documentation inherited.
      public: virtual RenderEngine *Engine() const override;

      // Documentation inherited.
      public: virtual VisualPtr RootVisual() const override;

      // Documentation inherited.
      public: virtual math::Color AmbientLight() const override;

      // Documentation inherited.
      public: virtual void SetAmbientLight(const math::Color &_color) override;

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void Clear() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual void SetSkyEnabled(bool _enabled) override;

      // Documentation inherited
      public: virtual bool SkyEnabled() const override;

      /// \brief Get a pointer to the ogre scene manager
      /// \return Pointer to the ogre scene manager
      public: virtual Ogre::SceneManager *OgreSceneManager() const;

      /// \cond PRIVATE
      /// \internal
      /// \brief Mark shadows dirty to rebuild compostior shadow node
      /// This is set when the number of shadow casting lighst changes
      /// \param[in] _dirty True to mark shadows are dirty
      /// \sa SetShadowsDirty
      public: void SetShadowsDirty(bool _dirty);

      /// \internal
      /// \brief Get whether shadows are dirty
      /// \return True if the number of shadow casting lights changed
      /// \sa ShadowsDirty
      public: bool ShadowsDirty() const;
      /// \endcond

      // Documentation inherited
      protected: virtual bool LoadImpl() override;

      // Documentation inherited
      protected: virtual bool InitImpl() override;

      // Documentation inherited
      protected: virtual LightVisualPtr CreateLightVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual DirectionalLightPtr CreateDirectionalLightImpl(
                     unsigned int _id, const std::string &_name) override;

      // Documentation inherited
      protected: virtual PointLightPtr CreatePointLightImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual SpotLightPtr CreateSpotLightImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual CameraPtr CreateCameraImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual DepthCameraPtr CreateDepthCameraImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual ThermalCameraPtr CreateThermalCameraImpl(
                     unsigned int _id, const std::string &_name) override;

      // Documentation inherited
      protected: virtual GpuRaysPtr CreateGpuRaysImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual VisualPtr CreateVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual ArrowVisualPtr CreateArrowVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual AxisVisualPtr CreateAxisVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual GizmoVisualPtr CreateGizmoVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual GeometryPtr CreateBoxImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual GeometryPtr CreateConeImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual GeometryPtr CreateCylinderImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual GeometryPtr CreatePlaneImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual GeometryPtr CreateSphereImpl(unsigned int _id,
                     const std::string &_name) override;

      /// \brief Create a mesh object based on its name
      /// \param[in] _id Unique Id to assign to the mesh
      /// \param[in] _name Name to assign to the mesh
      /// \param[in] _meshName Name of the mesh to create
      protected: virtual MeshPtr CreateMeshImpl(unsigned int _id,
                     const std::string &_name, const std::string &_meshName);

      // Documentation inherited
      protected: virtual MeshPtr CreateMeshImpl(unsigned int _id,
                     const std::string &_name, const MeshDescriptor &_desc)
                     override;

      // Documentation inherited
      protected: virtual HeightmapPtr CreateHeightmapImpl(unsigned int _id,
                   const std::string &_name, const HeightmapDescriptor &_desc)
                   override;

      // Documentation inherited
      protected: virtual GridPtr CreateGridImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual MarkerPtr CreateMarkerImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual LidarVisualPtr CreateLidarVisualImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual WireBoxPtr CreateWireBoxImpl(unsigned int _id,
                     const std::string &_name) override;

      // Documentation inherited
      protected: virtual TextPtr CreateTextImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual MaterialPtr CreateMaterialImpl(unsigned int _id,
                     const std::string &_name) override;

      protected: virtual RenderTexturePtr CreateRenderTextureImpl(
                     unsigned int _id, const std::string &_name) override;

      // Documentation inherited.
      protected: virtual RenderWindowPtr CreateRenderWindowImpl(
                     unsigned int _id, const std::string &_name) override;

      // Documentation inherited
      protected: virtual RayQueryPtr CreateRayQueryImpl(
                     unsigned int _id, const std::string &_name) override;

      // Documentation inherited
      protected: virtual ParticleEmitterPtr CreateParticleEmitterImpl(
                     unsigned int _id, const std::string &_name) override;

      /// \brief Helper function to initialize an ogre2 object
      /// \param[in] _object Ogre2 object that will be initialized
      /// \param[in] _id Unique Id to assign to the object
      /// \param[in] _name Name to assign to the object
      protected: virtual bool InitObject(Ogre2ObjectPtr _object,
                     unsigned int _id, const std::string &_name);

      // Documentation inherited
      protected: virtual LightStorePtr Lights() const override;

      // Documentation inherited
      protected: virtual SensorStorePtr Sensors() const override;

      // Documentation inherited
      protected: virtual VisualStorePtr Visuals() const override;

      // Documentation inherited
      protected: virtual MaterialMapPtr Materials() const override;

      /// \brief Create the GL context
      private: void CreateContext();

      /// \brief Create the root visual in the scene
      private: void CreateRootVisual();

      /// \brief Create the mesh factory used to generate ogre meshes
      private: void CreateMeshFactory();

      /// \brief Create the vaiours storage objects
      private: void CreateStores();

      /// \brief Create a shared pointer to self
      private: Ogre2ScenePtr SharedThis();

      /// \brief Root visual in the scene
      protected: Ogre2VisualPtr rootVisual;

      /// \brief Mesh factory for generating ogre meshes
      protected: Ogre2MeshFactoryPtr meshFactory;

      /// \brief A list of ogre sensors, e.g. cameras
      protected: Ogre2SensorStorePtr sensors;

      /// \brief A list of ogre visuals
      protected: Ogre2VisualStorePtr visuals;

      /// \brief A list of ogre lights
      protected: Ogre2LightStorePtr lights;

      /// \brief A list of ogre materials
      protected: Ogre2MaterialMapPtr materials;

      /// \brief Pointer to the ogre scene manager
      protected: Ogre::SceneManager *ogreSceneManager = nullptr;

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2ScenePrivate> dataPtr;

      /// \brief Make the render engine our friend
      private: friend class Ogre2RenderEngine;
    };
    }
  }
}
#endif
