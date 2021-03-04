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

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/Filesystem.hh>

#include "test_config.h"  // NOLINT(build/include)

#include "ignition/rendering/GpuRays.hh"
#include "ignition/rendering/ParticleEmitter.hh"
#include "ignition/rendering/RenderEngine.hh"
#include "ignition/rendering/RenderingIface.hh"
#include "ignition/rendering/Scene.hh"

#define LASER_TOL 2e-4
#define DOUBLE_TOL 1e-6

// vertical range values seem to be less accurate
#define VERTICAL_LASER_TOL 1e-3

#define WAIT_TIME 0.02

using namespace ignition;
using namespace rendering;

void OnNewGpuRaysFrame(float *_scanDest, const float *_scan,
                  unsigned int _width, unsigned int _height,
                  unsigned int _channels,
                  const std::string &/*_format*/)
{
  float f;
  int size =  _width * _height * _channels;
  memcpy(_scanDest, _scan, size * sizeof(f));
}

class GpuRaysTest: public testing::Test,
                  public testing::WithParamInterface<const char *>
{
  // Test and verify gpu rays properties setters and getters
  public: void Configure(const std::string &_renderEngine);

  // Test boxes detection
  public: void RaysUnitBox(const std::string &_renderEngine);

  // Test vertical measurements
  public: void LaserVertical(const std::string &_renderEngine);

  // Test detection of particles
  public: void RaysParticles(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
/// \brief Test GPU rays configuraions
void GpuRaysTest::Configure(const std::string &_renderEngine)
{
  if (_renderEngine == "optix")
  {
    igndbg << "GpuRays not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  GpuRaysPtr gpuRays = scene->CreateGpuRays();
  ASSERT_TRUE(gpuRays != nullptr);
  root->AddChild(gpuRays);

  // set gpu rays caster initial pose
  math::Vector3d initPos(-2, 0.0, 5.0);
  math::Quaterniond initRot = math::Quaterniond::Identity;
  gpuRays->SetWorldPosition(initPos);
  EXPECT_EQ(initPos, gpuRays->WorldPosition());
  EXPECT_EQ(initRot, gpuRays->WorldRotation());

  // The following tests all the getters and setters
  {
    gpuRays->SetNearClipPlane(0.1);
    EXPECT_NEAR(gpuRays->NearClipPlane(), 0.1, 1e-6);

    gpuRays->SetFarClipPlane(100.0);
    EXPECT_NEAR(gpuRays->FarClipPlane(), 100, 1e-6);

    gpuRays->SetIsHorizontal(false);
    EXPECT_FALSE(gpuRays->IsHorizontal());

    gpuRays->SetNearClipPlane(0.04);
    EXPECT_NEAR(gpuRays->NearClipPlane(), 0.04, 1e-6);

    gpuRays->SetFarClipPlane(5.4);
    EXPECT_NEAR(gpuRays->FarClipPlane(), 05.4, 1e-6);

    gpuRays->SetAngleMin(-1.47);
    EXPECT_NEAR(gpuRays->AngleMin().Radian(), -1.47, 1e-6);

    gpuRays->SetAngleMax(1.56);
    EXPECT_NEAR(gpuRays->AngleMax().Radian(), 1.56, 1e-6);

    gpuRays->SetVerticalAngleMin(-0.32);
    EXPECT_NEAR(gpuRays->VerticalAngleMin().Radian(), -0.32, 1e-6);

    gpuRays->SetVerticalAngleMax(1.58);
    EXPECT_NEAR(gpuRays->VerticalAngleMax().Radian(), 1.58, 1e-6);

    EXPECT_EQ(gpuRays->Clamp(), false);
    gpuRays->SetClamp(true);
    EXPECT_EQ(gpuRays->Clamp(), true);

    gpuRays->SetVerticalRayCount(67);
    EXPECT_NEAR(gpuRays->VerticalRayCount(), 67, 1e-6);

    EXPECT_DOUBLE_EQ(1.0, gpuRays->HorizontalResolution());
    EXPECT_DOUBLE_EQ(1.0, gpuRays->VerticalResolution());

    gpuRays->SetHorizontalResolution(0.1);
    gpuRays->SetVerticalResolution(10.5);
    EXPECT_DOUBLE_EQ(0.1, gpuRays->HorizontalResolution());
    EXPECT_DOUBLE_EQ(10.5, gpuRays->VerticalResolution());

    gpuRays->SetHorizontalResolution(-2.4);
    gpuRays->SetVerticalResolution(-0.8);
    EXPECT_DOUBLE_EQ(2.4, gpuRays->HorizontalResolution());
    EXPECT_DOUBLE_EQ(0.8, gpuRays->VerticalResolution());
  }

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}


/////////////////////////////////////////////////
/// \brief Test detection of different boxes
void GpuRaysTest::RaysUnitBox(const std::string &_renderEngine)
{
#ifdef __APPLE__
  std::cerr << "Skipping test for apple, see issue #35." << std::endl;
  return;
#endif

  if (_renderEngine == "optix")
  {
    igndbg << "GpuRays not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  // Test GPU rays with 3 boxes in the world.
  // First GPU rays at identity orientation, second at 90 degree roll
  // First place 2 of 3 boxes within range and verify range values.
  // then move all 3 boxes out of range and verify range values

  const double hMinAngle = -IGN_PI/2.0;
  const double hMaxAngle = IGN_PI/2.0;
  const double minRange = 0.1;
  const double maxRange = 10.0;
  const int hRayCount = 320;
  const int vRayCount = 1;

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  // Create first ray caster
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond::Identity);

  GpuRaysPtr gpuRays = scene->CreateGpuRays("gpu_rays_1");
  gpuRays->SetWorldPosition(testPose.Pos());
  gpuRays->SetWorldRotation(testPose.Rot());
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetRayCount(hRayCount);

  gpuRays->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays);

  // Create a second ray caster rotated
  ignition::math::Pose3d testPose2(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond(IGN_PI/2.0, 0, 0));

  GpuRaysPtr gpuRays2 = scene->CreateGpuRays("gpu_rays_2");
  gpuRays2->SetWorldPosition(testPose2.Pos());
  gpuRays2->SetWorldRotation(testPose2.Rot());
  gpuRays2->SetNearClipPlane(minRange);
  gpuRays2->SetFarClipPlane(maxRange);
  gpuRays2->SetClamp(true);
  gpuRays2->SetAngleMin(hMinAngle);
  gpuRays2->SetAngleMax(hMaxAngle);
  gpuRays2->SetRayCount(hRayCount);
  gpuRays2->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays2);

  // Laser retro test values
  double laserRetro1 = 2000;
  double laserRetro2 = 1000;
  std::string userDataKey = "laser_retro";

  // Create testing boxes
  // box in the center
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(3, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox1 = scene->CreateVisual("UnitBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  visualBox1->SetUserData(userDataKey, laserRetro1);
  root->AddChild(visualBox1);



  // box on the right of the first gpu rays caster
  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(0, -5, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox2 = scene->CreateVisual("UnitBox2");
  visualBox2->AddGeometry(scene->CreateBox());
  visualBox2->SetWorldPosition(box02Pose.Pos());
  visualBox2->SetWorldRotation(box02Pose.Rot());
  visualBox2->SetUserData(userDataKey, laserRetro2);
  root->AddChild(visualBox2);

  // box on the left of the rays caster 1 but out of range
  ignition::math::Pose3d box03Pose(
      ignition::math::Vector3d(0, maxRange + 1, 0.5),
      ignition::math::Quaterniond::Identity);
  VisualPtr visualBox3 = scene->CreateVisual("UnitBox3");
  visualBox3->AddGeometry(scene->CreateBox());
  visualBox3->SetWorldPosition(box03Pose.Pos());
  visualBox3->SetWorldRotation(box03Pose.Rot());
  root->AddChild(visualBox3);

  // Verify rays caster 1 range readings
  // listen to new gpu rays frames
  unsigned int channels = gpuRays->Channels();
  float *scan = new float[hRayCount * vRayCount * channels];
  common::ConnectionPtr c =
    gpuRays->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  gpuRays->Update();

  int mid = static_cast<int>(hRayCount/2) * channels;
  int last = (hRayCount - 1) * channels;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 = abs(box01Pose.Pos().X()) - unitBoxSize/2;
  double expectedRangeAtMidPointBox2 = abs(box02Pose.Pos().Y()) - unitBoxSize/2;

  // rays caster 1 should see box01 and box02
  EXPECT_NEAR(scan[mid], expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_NEAR(scan[0], expectedRangeAtMidPointBox2, LASER_TOL);
  EXPECT_DOUBLE_EQ(scan[last], ignition::math::INF_D);

  // rays cater should see box01 with laser retro value set to laserRetro1
  // and box02 with laser retro value set to laserRetro2
  EXPECT_NEAR(scan[mid+1], laserRetro1, 5.0);
  EXPECT_NEAR(scan[0+1], laserRetro2, 5.0);
  EXPECT_DOUBLE_EQ(scan[last+1], 0.0);

  // Verify rays caster 2 range readings
  // listen to new gpu rays frames
  float *scan2 = new float[hRayCount * vRayCount * 3];

  gpuRays2->Update();
  // Test Copy method instead of using the callback for the second rays caster
  gpuRays2->Copy(scan2);

  // Only box01 should be visible to rays caster 2
  EXPECT_DOUBLE_EQ(scan2[0], maxRange);
  EXPECT_NEAR(scan2[mid], expectedRangeAtMidPointBox1, LASER_TOL);
  EXPECT_DOUBLE_EQ(scan2[last], maxRange);

  // Move all boxes out of range
  visualBox1->SetWorldPosition(
      ignition::math::Vector3d(maxRange + 1, 0, 0));
  visualBox1->SetWorldRotation(box01Pose.Rot());
  visualBox2->SetWorldPosition(
      ignition::math::Vector3d(0, -(maxRange + 1), 0));
  visualBox2->SetWorldRotation(box02Pose.Rot());

  gpuRays->Update();
  gpuRays2->Update();
  gpuRays2->Copy(scan2);

  for (int i = 0; i < gpuRays->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(scan[i * 3], ignition::math::INF_D);

  for (int i = 0; i < gpuRays2->RayCount(); ++i)
    EXPECT_DOUBLE_EQ(scan2[i * 3], maxRange);

  c.reset();

  delete [] scan;
  delete [] scan2;

  scan = nullptr;
  scan2 = nullptr;

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test GPU rays vertical component
void GpuRaysTest::LaserVertical(const std::string &_renderEngine)
{
#ifdef __APPLE__
  std::cerr << "Skipping test for apple, see issue #35." << std::endl;
  return;
#endif

  if (_renderEngine == "optix")
  {
    igndbg << "GpuRays not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  // Test a rays that has a vertical range component.
  // Place a box within range and verify range values,
  // then move the box out of range and verify range values

  double hMinAngle = -IGN_PI/2.0;
  double hMaxAngle = IGN_PI/2.0;
  double vMinAngle = -IGN_PI/4.0;
  double vMaxAngle = IGN_PI/4.0;
  double minRange = 0.1;
  double maxRange = 5.0;
  unsigned int hRayCount = 640;
  unsigned int vRayCount = 4;

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  // Create first ray caster
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0.25, 0, 0.5),
      ignition::math::Quaterniond::Identity);

  GpuRaysPtr gpuRays = scene->CreateGpuRays("vertical_gpu_rays");
  gpuRays->SetWorldPosition(testPose.Pos());
  gpuRays->SetWorldRotation(testPose.Rot());
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetVerticalAngleMin(vMinAngle);
  gpuRays->SetVerticalAngleMax(vMaxAngle);
  gpuRays->SetRayCount(hRayCount);
  gpuRays->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays);

  // Create testing boxes
  // box in front of ray sensor
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(1, 0, 0.5),
      ignition::math::Quaterniond::Identity);
  VisualPtr visualBox1 = scene->CreateVisual("VerticalTestBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  unsigned int channels = gpuRays->Channels();
  float *scan = new float[hRayCount * vRayCount * channels];
  common::ConnectionPtr c =
    gpuRays->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  gpuRays->Update();

  unsigned int mid = hRayCount * channels / 2;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPoint = box01Pose.Pos().X() - unitBoxSize/2
      - testPose.Pos().X();

  double vAngleStep = (vMaxAngle - vMinAngle) / (vRayCount-1);
  double angleStep = vMinAngle;

  // all vertical laser planes should sense box
  for (unsigned int i = 0; i < vRayCount; ++i)
  {
    double expectedRange = expectedRangeAtMidPoint / cos(angleStep);

    EXPECT_NEAR(scan[i * hRayCount * channels + mid],
        expectedRange, VERTICAL_LASER_TOL);

    angleStep += vAngleStep;

    // check that the values in the extremes are infinity
    EXPECT_DOUBLE_EQ(scan[i * hRayCount * channels],
        ignition::math::INF_D);
    EXPECT_DOUBLE_EQ(scan[(i * hRayCount + (hRayCount - 1)) * channels],
        ignition::math::INF_D);
  }

  // Move box out of range
  visualBox1->SetWorldPosition(
      ignition::math::Vector3d(maxRange + 1, 0, 0));
  visualBox1->SetWorldRotation(
      ignition::math::Quaterniond::Identity);

  // wait for a few more laser scans
  gpuRays->Update();

  for (int j = 0; j < gpuRays->VerticalRayCount(); ++j)
  {
    for (int i = 0; i < gpuRays->RayCount(); ++i)
    {
      EXPECT_DOUBLE_EQ(scan[j * gpuRays->RayCount() * channels+ i * channels],
          ignition::math::INF_D);
    }
  }

  c.reset();

  delete [] scan;
  scan = nullptr;

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
/// \brief Test detection of particles
void GpuRaysTest::RaysParticles(const std::string &_renderEngine)
{
#ifdef __APPLE__
  std::cerr << "Skipping test for apple, see issue #35." << std::endl;
  return;
#endif

  if (_renderEngine != "ogre2")
  {
    igndbg << "GpuRays with particle effect is not supported yet in rendering "
           << "engine: " << _renderEngine << std::endl;
    return;
  }

  // Test GPU ray with 3 boxes in the world.
  // Add noise in btewen GPU ray and box in the center

  const double hMinAngle = -IGN_PI / 2.0;
  const double hMaxAngle = IGN_PI / 2.0;
  const double minRange = 0.1;
  const double maxRange = 10.0;
  const int hRayCount = 320;
  const int vRayCount = 1;

  // create and populate scene
  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    igndbg << "Engine '" << _renderEngine
              << "' is not supported" << std::endl;
    return;
  }

  ScenePtr scene = engine->CreateScene("scene");
  ASSERT_TRUE(scene != nullptr);

  VisualPtr root = scene->RootVisual();

  // Create ray caster
  ignition::math::Pose3d testPose(ignition::math::Vector3d(0, 0, 0.1),
      ignition::math::Quaterniond::Identity);

  GpuRaysPtr gpuRays = scene->CreateGpuRays("gpu_rays_1");
  gpuRays->SetWorldPosition(testPose.Pos());
  gpuRays->SetWorldRotation(testPose.Rot());
  gpuRays->SetNearClipPlane(minRange);
  gpuRays->SetFarClipPlane(maxRange);
  gpuRays->SetAngleMin(hMinAngle);
  gpuRays->SetAngleMax(hMaxAngle);
  gpuRays->SetRayCount(hRayCount);

  gpuRays->SetVerticalRayCount(vRayCount);
  root->AddChild(gpuRays);

  // Create testing boxes
  // box in the center
  ignition::math::Pose3d box01Pose(ignition::math::Vector3d(3, 0, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox1 = scene->CreateVisual("UnitBox1");
  visualBox1->AddGeometry(scene->CreateBox());
  visualBox1->SetWorldPosition(box01Pose.Pos());
  visualBox1->SetWorldRotation(box01Pose.Rot());
  root->AddChild(visualBox1);

  // box on the right of the first gpu rays caster
  ignition::math::Pose3d box02Pose(ignition::math::Vector3d(0, -5, 0.5),
                                   ignition::math::Quaterniond::Identity);
  VisualPtr visualBox2 = scene->CreateVisual("UnitBox2");
  visualBox2->AddGeometry(scene->CreateBox());
  visualBox2->SetWorldPosition(box02Pose.Pos());
  visualBox2->SetWorldRotation(box02Pose.Rot());
  root->AddChild(visualBox2);

  // box on the left of the rays caster 1 but out of range
  ignition::math::Pose3d box03Pose(
      ignition::math::Vector3d(0, maxRange + 1, 0.5),
      ignition::math::Quaterniond::Identity);
  VisualPtr visualBox3 = scene->CreateVisual("UnitBox3");
  visualBox3->AddGeometry(scene->CreateBox());
  visualBox3->SetWorldPosition(box03Pose.Pos());
  visualBox3->SetWorldRotation(box03Pose.Rot());
  root->AddChild(visualBox3);

  // create particle emitter between sensor and box in the center
  ignition::math::Vector3d particlePosition(1.0, 0, 0);
  ignition::math::Vector3d particleSize(0.2, 0.2, 0.2);
  ignition::rendering::ParticleEmitterPtr emitter =
      scene->CreateParticleEmitter();
  emitter->SetLocalPosition(particlePosition);
  emitter->SetParticleSize(particleSize);
  emitter->SetRate(100);
  emitter->SetLifetime(2);
  emitter->SetVelocityRange(0.1, 0.1);
  emitter->SetScaleRate(0.1);
  emitter->SetColorRange(ignition::math::Color::Red,
      ignition::math::Color::Black);
  emitter->SetEmitting(true);
  root->AddChild(emitter);

  // Verify rays caster 1 range readings
  // listen to new gpu rays frames
  unsigned int channels = gpuRays->Channels();
  float *scan = new float[hRayCount * vRayCount * channels];
  common::ConnectionPtr c =
    gpuRays->ConnectNewGpuRaysFrame(
        std::bind(&::OnNewGpuRaysFrame, scan,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));


  int mid = static_cast<int>(hRayCount / 2) * channels;
  int last = (hRayCount - 1) * channels;
  double unitBoxSize = 1.0;
  double expectedRangeAtMidPointBox1 =
      abs(box01Pose.Pos().X()) - unitBoxSize / 2;
  double expectedRangeAtMidPointBox2 =
      abs(box02Pose.Pos().Y()) - unitBoxSize / 2;

  // set a larger tol for particle range
  // depth noise is computed based on particle size
  double laserNoiseTol = particleSize.X() + particleSize.X() * 0.5;
  double expectedParticleRange = particlePosition.X();

  // update 100 frames. There should be a descent chance that we will see both
  // a particle hit and miss in the readings returned by the sensor
  unsigned int particleHitCount = 0u;
  unsigned int particleMissCount = 0u;
  for (unsigned int i = 0u; i < 100u; ++i)
  {
    gpuRays->Update();

    // sensor should see ether a particle or box01
    double particleRange = static_cast<double>(scan[mid]);
    bool particleHit = ignition::math::equal(
        expectedParticleRange, particleRange, laserNoiseTol);
    bool particleMiss = ignition::math::equal(
        expectedRangeAtMidPointBox1, particleRange, LASER_TOL);
    EXPECT_TRUE(particleHit || particleMiss)
        << "actual vs expected particle range: "
        << particleRange << " vs " << expectedParticleRange;

    particleHitCount += particleHit ? 1u : 0u;
    particleMissCount += particleMiss ? 1u : 0u;

    // sensor should see box02 without noise or scatter effect
    EXPECT_NEAR(expectedRangeAtMidPointBox2, scan[0], LASER_TOL);

    // sensor should not see box03 as it is out of range
    EXPECT_DOUBLE_EQ(ignition::math::INF_D, scan[last]);
  }

  // there should be at least one hit
  EXPECT_GT(particleHitCount, 0u);
  // there should be at least one miss
  EXPECT_GT(particleMissCount, 0u);

  c.reset();

  delete [] scan;

  scan = nullptr;

  // Clean up
  engine->DestroyScene(scene);
  rendering::unloadEngine(engine->Name());
}
/////////////////////////////////////////////////
TEST_P(GpuRaysTest, Configure)
{
  Configure(GetParam());
}

/////////////////////////////////////////////////
TEST_P(GpuRaysTest, RaysUnitBox)
{
  RaysUnitBox(GetParam());
}

/////////////////////////////////////////////////
TEST_P(GpuRaysTest, LaserVertical)
{
  LaserVertical(GetParam());
}

/////////////////////////////////////////////////
TEST_P(GpuRaysTest, RaysParticles)
{
  RaysParticles(GetParam());
}

INSTANTIATE_TEST_CASE_P(GpuRays, GpuRaysTest,
    RENDER_ENGINE_VALUES,
    ignition::rendering::PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
