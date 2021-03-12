/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#if __APPLE__
  #include <OpenGL/gl.h>
  #include <OpenGL/OpenGL.h>
  #include <GLUT/glut.h>
#else
  #include <GL/glew.h>
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif

#if !defined(__APPLE__) && !defined(_WIN32)
  #include <GL/glx.h>
#endif

#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Image.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/ThermalCamera.hh>

#include "GlutWindow.hh"

#define KEY_ESC 27
#define KEY_TAB  9

//////////////////////////////////////////////////
unsigned int imgw = 0;
unsigned int imgh = 0;

std::vector<ir::CameraPtr> g_cameras;
ir::CameraPtr g_camera;
unsigned int g_cameraIndex = 0;
ir::ImagePtr g_image;
ignition::common::ConnectionPtr g_connection;


bool g_initContext = false;

#if __APPLE__
  CGLContextObj g_context;
  CGLContextObj g_glutContext;
#elif _WIN32
#else
  GLXContext g_context;
  Display *g_display;
  GLXDrawable g_drawable;
  GLXContext g_glutContext;
  Display *g_glutDisplay;
  GLXDrawable g_glutDrawable;
#endif

//////////////////////////////////////////////////
void OnNewThermalFrame(const uint16_t *_scan,
                    unsigned int _width, unsigned int _height,
                    unsigned int /*_channels*/,
                    const std::string &_format)
{
  // get min and max of temperature values
  uint16_t min = std::numeric_limits<uint16_t>::max();
  uint16_t max = 0;
  for (unsigned int i = 0; i < _height * _width; ++i)
  {
    uint16_t temp = _scan[i];
    if (temp > max)
      max = temp;
    if (temp < min)
      min = temp;
  }

  // convert temperature to grayscale image
  double range = static_cast<double>(max - min);
  if (ignition::math::equal(range, 0.0))
    range = 1.0;
  unsigned char *data = g_image->Data<unsigned char>();
  for (unsigned int i = 0; i < _height; ++i)
  {
    for (unsigned int j = 0; j < _width; ++j)
    {
      uint16_t temp = _scan[i*_width + j];
      double t = static_cast<double>(temp-min) / range;
      int r = 255*t;
      int g = r;
      int b = r;
      int index = i*_width*3 + j*3;
      data[index] = r;
      data[index+1] = g;
      data[index+2] = b;
    }
  }
}

//////////////////////////////////////////////////
void displayCB()
{
#if __APPLE__
  CGLSetCurrentContext(g_context);
#elif _WIN32
#else
  if (g_display)
  {
    glXMakeCurrent(g_display, g_drawable, g_context);
  }
#endif

  g_cameras[g_cameraIndex]->Update();

#if __APPLE__
  CGLSetCurrentContext(g_glutContext);
#elif _WIN32
#else
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
#endif

  unsigned char *data = g_image->Data<unsigned char>();

  glClearColor(0.5, 0.5, 0.5, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPixelZoom(1, -1);
  glRasterPos2f(-1, 1);
  glDrawPixels(imgw, imgh, GL_RGB, GL_UNSIGNED_BYTE, data);

  glutSwapBuffers();
}

//////////////////////////////////////////////////
void idleCB()
{
  glutPostRedisplay();
}

//////////////////////////////////////////////////
void keyboardCB(unsigned char _key, int, int)
{
  if (_key == KEY_ESC || _key == 'q' || _key == 'Q')
  {
    exit(0);
  }
  else if (_key == KEY_TAB)
  {
    g_cameraIndex = (g_cameraIndex + 1) % g_cameras.size();
  }
}

//////////////////////////////////////////////////
void initCamera(ir::CameraPtr _camera)
{
  g_camera = _camera;
  imgw = g_camera->ImageWidth();
  imgh = g_camera->ImageHeight();
  ir::Image image = g_camera->CreateImage();
  g_image = std::make_shared<ir::Image>(image);

  ir::ThermalCameraPtr camera = std::dynamic_pointer_cast<ir::ThermalCamera>(
      g_camera);

  // callback when new thermal frame is received.
  g_connection = camera->ConnectNewThermalFrame(
      std::bind(OnNewThermalFrame,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  g_camera->Update();
}

//////////////////////////////////////////////////
void initContext()
{
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(imgw, imgh);
  glutCreateWindow("Thermal Camera");
  glutDisplayFunc(displayCB);
  glutIdleFunc(idleCB);
  glutKeyboardFunc(keyboardCB);
}

//////////////////////////////////////////////////
void printUsage()
{
  std::cout << "===============================" << std::endl;
  std::cout << "  TAB - Switch render engines  " << std::endl;
  std::cout << "  ESC - Exit                   " << std::endl;
  std::cout << "===============================" << std::endl;
}

//////////////////////////////////////////////////
void run(std::vector<ir::CameraPtr> _cameras)
{
  if (_cameras.empty())
  {
    ignerr << "No cameras found. Scene will not be rendered" << std::endl;
    return;
  }

#if __APPLE__
  g_context = CGLGetCurrentContext();
#elif _WIN32
#else
  g_context = glXGetCurrentContext();
  g_display = glXGetCurrentDisplay();
  g_drawable = glXGetCurrentDrawable();
#endif

#if __APPLE__
  g_glutContext = CGLGetCurrentContext();
#elif _WIN32
#else
  auto dummyDisplay = XOpenDisplay(0);
  g_glutDisplay = static_cast<Display*>(dummyDisplay);//glXGetCurrentDisplay();

  // create X11 visual
  int screenId = DefaultScreen(g_glutDisplay);

  std::cerr << "screenId " << screenId << '\n';

  int attributeList[] = {
    GLX_RENDER_TYPE, GLX_RGBA_BIT,
    GLX_DOUBLEBUFFER, True,
    GLX_DEPTH_SIZE, 16,
    GLX_STENCIL_SIZE, 8,
    None
  };

  int nelements = 0;

  auto dummyFBConfigs =
      glXChooseFBConfig(g_glutDisplay, screenId, attributeList, &nelements);

  g_glutDrawable = XCreateSimpleWindow(g_glutDisplay,
      RootWindow(dummyDisplay, screenId), 0, 0, 1, 1, 0, 0, 0);

  PFNGLXCREATECONTEXTATTRIBSARBPROC glXCreateContextAttribsARB = 0;
  glXCreateContextAttribsARB =
      (PFNGLXCREATECONTEXTATTRIBSARBPROC)
      glXGetProcAddress((const GLubyte*)"glXCreateContextAttribsARB");

  int contextAttribs[] = {
    GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
    GLX_CONTEXT_MINOR_VERSION_ARB, 3,
    GLX_CONTEXT_FLAGS_ARB        , GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
    None
  };
  g_glutContext =
      glXCreateContextAttribsARB(g_glutDisplay,
                                dummyFBConfigs[0], nullptr,
                                1, contextAttribs);

  g_glutContext = static_cast<GLXContext>(g_glutContext);

  // g_glutDrawable = glXGetCurrentDrawable();
  // g_glutContext = glXGetCurrentContext();
#endif
  g_cameras = _cameras;
  initCamera(_cameras[0]);
  initContext();
  printUsage();
  glutMainLoop();
}
