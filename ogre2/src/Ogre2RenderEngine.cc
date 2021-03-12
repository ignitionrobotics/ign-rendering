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

// Not Apple or Windows
#if !defined(__APPLE__) && !defined(_WIN32)
# include <X11/Xlib.h>
# include <X11/Xutil.h>
# include <GL/glx.h>
# include <GL/glxext.h>
#endif

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif
#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>

#include <ignition/plugin/Register.hh>

#include "ignition/rendering/RenderEngineManager.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Storage.hh"


class ignition::rendering::Ogre2RenderEnginePrivate
{
#if !defined(__APPLE__) && !defined(_WIN32)
  public: GLXFBConfig* dummyFBConfigs = nullptr;
#endif

  /// \brief A list of supported fsaa levels
  public: std::vector<unsigned int> fsaaLevels;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2RenderEnginePlugin::Ogre2RenderEnginePlugin()
{
}

//////////////////////////////////////////////////
std::string Ogre2RenderEnginePlugin::Name() const
{
  return Ogre2RenderEngine::Instance()->Name();
}

//////////////////////////////////////////////////
RenderEngine *Ogre2RenderEnginePlugin::Engine() const
{
  return Ogre2RenderEngine::Instance();
}

//////////////////////////////////////////////////
Ogre2RenderEngine::Ogre2RenderEngine() :
  dataPtr(new Ogre2RenderEnginePrivate)
{
  this->dummyDisplay = nullptr;
  this->dummyContext = 0;
  this->dummyWindowId = 0;

  std::string ogrePath = std::string(OGRE2_RESOURCE_PATH);
  this->ogrePaths.push_back(ogrePath);

  const char *env = std::getenv("OGRE2_RESOURCE_PATH");
  if (env)
    this->ogrePaths.push_back(std::string(env));

#ifdef __APPLE__
  // on OSX the plugins may be placed in the parent lib directory
  if (ogrePath.rfind("OGRE") == ogrePath.size()-4u)
    this->ogrePaths.push_back(ogrePath.substr(0, ogrePath.size()-5));
#endif
}

//////////////////////////////////////////////////
Ogre2RenderEngine::~Ogre2RenderEngine()
{
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::Destroy()
{
  BaseRenderEngine::Destroy();

  if (this->scenes)
  {
    this->scenes->RemoveAll();
  }

  delete this->ogreOverlaySystem;
  this->ogreOverlaySystem = nullptr;

  if (ogreRoot)
  {
    try
    {
      // TODO(anyone): do we need to catch segfault on delete?
      delete this->ogreRoot;
    }
    catch (...)
    {
    }
    this->ogreRoot = nullptr;
  }

  delete this->ogreLogManager;
  this->ogreLogManager = nullptr;

#if not (__APPLE__ || _WIN32)
  if (this->dummyDisplay)
  {
    Display *x11Display = static_cast<Display*>(this->dummyDisplay);
    GLXContext x11Context = static_cast<GLXContext>(this->dummyContext);
    glXDestroyContext(x11Display, x11Context);
    XDestroyWindow(x11Display, this->dummyWindowId);
    XCloseDisplay(x11Display);
    this->dummyDisplay = nullptr;
    XFree(this->dataPtr->dummyFBConfigs);
    this->dataPtr->dummyFBConfigs = nullptr;
  }
#endif
}

//////////////////////////////////////////////////
bool Ogre2RenderEngine::IsEnabled() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
std::string Ogre2RenderEngine::Name() const
{
  return "ogre2";
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::AddResourcePath(const std::string &_uri)
{
  if (_uri == "__default__" || _uri.empty())
    return;

  std::string path = common::findFilePath(_uri);

  if (path.empty())
  {
    ignerr << "URI doesn't exist[" << _uri << "]\n";
    return;
  }

  this->resourcePaths.push_back(path);

  try
  {
    if (!Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(
          path, "General"))
    {
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          path, "FileSystem", "General", true);

      Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
          "General", false);
      // Parse all material files in the path if any exist
      if (common::isDirectory(path))
      {
        std::vector<std::string> paths;

        common::DirIter endIter;
        for (common::DirIter dirIter(path); dirIter != endIter; ++dirIter)
        {
          paths.push_back(*dirIter);
        }
        std::sort(paths.begin(), paths.end());

        // Iterate over all the models in the current ign-rendering path
        for (auto dIter = paths.begin(); dIter != paths.end(); ++dIter)
        {
          std::string fullPath = *dIter;
          std::string matExtension = fullPath.substr(fullPath.size()-9);
          if (matExtension == ".material")
          {
            Ogre::DataStreamPtr stream =
              Ogre::ResourceGroupManager::getSingleton().openResource(
                  fullPath, "General");

            // There is a material file under there somewhere, read the thing in
            try
            {
              Ogre::MaterialManager::getSingleton().parseScript(
                  stream, "General");
              Ogre::MaterialPtr matPtr =
                Ogre::MaterialManager::getSingleton().getByName(
                    fullPath);

              if (!matPtr.isNull())
              {
                // is this necessary to do here? Someday try it without
                matPtr->compile();
                matPtr->load();
              }
            }
            catch(Ogre::Exception& e)
            {
              ignerr << "Unable to parse material file[" << fullPath << "]\n";
            }
            stream->close();
          }
        }
      }
    }
  }
  catch(Ogre::Exception &_e)
  {
    ignerr << "Unable to load Ogre Resources.\nMake sure the"
        "resources path in the world file is set correctly." << std::endl;
  }
}

//////////////////////////////////////////////////
Ogre::Root *Ogre2RenderEngine::OgreRoot() const
{
  return this->ogreRoot;
}

//////////////////////////////////////////////////
ScenePtr Ogre2RenderEngine::CreateSceneImpl(unsigned int _id,
    const std::string &_name)
{
  Ogre2ScenePtr scene = Ogre2ScenePtr(new Ogre2Scene(_id, _name));
  this->scenes->Add(scene);
  return scene;
}

//////////////////////////////////////////////////
SceneStorePtr Ogre2RenderEngine::Scenes() const
{
  return this->scenes;
}

//////////////////////////////////////////////////
bool Ogre2RenderEngine::LoadImpl(
    const std::map<std::string, std::string> &_params)
{
  // parse params
  auto it = _params.find("useCurrentGLContext");
  if (it != _params.end())
    std::istringstream(it->second) >> this->useCurrentGLContext;

  try
  {
    this->LoadAttempt();
    this->loaded = true;
    return true;
  }
  catch (Ogre::Exception &ex)
  {
    ignerr << ex.what() << std::endl;
    return false;
  }
  catch (...)
  {
    ignerr << "Failed to load render-engine" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
bool Ogre2RenderEngine::InitImpl()
{
  try
  {
    this->InitAttempt();
    return true;
  }
  catch (Ogre::Exception &ex)
  {
    ignerr << ex.what() << std::endl;
    return false;
  }
  catch (...)
  {
    ignerr << "Failed to initialize render-engine" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::LoadAttempt()
{
  this->CreateLogger();
  if (!this->useCurrentGLContext)
    this->CreateContext();
  this->CreateRoot();
  this->CreateOverlay();
  this->LoadPlugins();
  this->CreateRenderSystem();
  this->ogreRoot->initialise(false);
  this->CreateRenderWindow();
  this->CreateResources();
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::CreateLogger()
{
  // create log file path
  std::string logPath;
  ignition::common::env(IGN_HOMEDIR, logPath);
  logPath = common::joinPaths(logPath, ".ignition", "rendering");
  common::createDirectories(logPath);
  logPath = common::joinPaths(logPath, "ogre2.log");

  // create actual log
  this->ogreLogManager = new Ogre::LogManager();
  this->ogreLogManager->createLog(logPath, true, false, false);
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::CreateContext()
{
#if not (__APPLE__ || _WIN32)
  // create X11 display
  this->dummyDisplay = XOpenDisplay(0);
  Display *x11Display = static_cast<Display*>(this->dummyDisplay);

  if (!this->dummyDisplay)
  {
    ignerr << "Unable to open display: " << XDisplayName(0) << std::endl;
    return;
  }

  // create X11 visual
  int screenId = DefaultScreen(x11Display);

  int attributeList[] = {
    GLX_RENDER_TYPE, GLX_RGBA_BIT,
    GLX_DOUBLEBUFFER, True,
    GLX_DEPTH_SIZE, 16,
    GLX_STENCIL_SIZE, 8,
    None
  };

  int nelements = 0;

  this->dataPtr->dummyFBConfigs =
      glXChooseFBConfig(x11Display, screenId, attributeList, &nelements);

  if (nelements <= 0)
  {
    ignerr << "Unable to create glx fbconfig" << std::endl;
    return;
  }

  // create X11 context
  this->dummyWindowId = XCreateSimpleWindow(x11Display,
      RootWindow(this->dummyDisplay, screenId), 0, 0, 1, 1, 0, 0, 0);

  PFNGLXCREATECONTEXTATTRIBSARBPROC glXCreateContextAttribsARB = 0;
  glXCreateContextAttribsARB =
      (PFNGLXCREATECONTEXTATTRIBSARBPROC)
      glXGetProcAddress((const GLubyte*)"glXCreateContextAttribsARB");

  if (glXCreateContextAttribsARB)
  {
    int contextAttribs[] = {
      GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
      GLX_CONTEXT_MINOR_VERSION_ARB, 3,
      GLX_CONTEXT_FLAGS_ARB        , GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
      None
    };
    this->dummyContext =
        glXCreateContextAttribsARB(x11Display,
                                  this->dataPtr->dummyFBConfigs[0], nullptr,
                                  1, contextAttribs);
  }
  else
  {
    ignwarn << "glXCreateContextAttribsARB() not found" << std::endl;
    this->dummyContext = glXCreateNewContext(x11Display,
                                             this->dataPtr->dummyFBConfigs[0],
                                             GLX_RGBA_TYPE, nullptr, 1);
  }

  GLXContext x11Context = static_cast<GLXContext>(this->dummyContext);

  if (!this->dummyContext)
  {
    ignerr << "Unable to create glx context" << std::endl;
    return;
  }

  // select X11 context
  glXMakeCurrent(x11Display, this->dummyWindowId, x11Context);
#endif
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::CreateRoot()
{
  try
  {
    this->ogreRoot = new Ogre::Root("", "", "");
  }
  catch (Ogre::Exception &ex)
  {
    ignerr << "Unable to create Ogre root" << std::endl;
  }
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::CreateOverlay()
{
  this->ogreOverlaySystem = new Ogre::v1::OverlaySystem();
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::LoadPlugins()
{
  for (auto iter = this->ogrePaths.begin();
       iter != this->ogrePaths.end(); ++iter)
  {
    std::string path(*iter);
    if (!common::isDirectory(path))
      continue;

    std::vector<std::string> plugins;
    std::vector<std::string>::iterator piter;

#ifdef __APPLE__
    std::string extension = ".dylib";
#elif _WIN32
    std::string extension = ".dll";
#else
    std::string extension = ".so";
#endif
    std::string p = common::joinPaths(path, "RenderSystem_GL3Plus");
    plugins.push_back(p);
    p = common::joinPaths(path, "Plugin_ParticleFX");
    plugins.push_back(p);

    for (piter = plugins.begin(); piter != plugins.end(); ++piter)
    {
      // check if plugin library exists
      std::string filename = *piter+extension;
      if (!common::exists(filename))
      {
        filename = filename + "." + std::string(OGRE2_VERSION);
        if (!common::exists(filename))
        {
          if ((*piter).find("RenderSystem") != std::string::npos)
          {
            ignerr << "Unable to find Ogre Plugin[" << *piter
                   << "]. Rendering will not be possible."
                   << "Make sure you have installed OGRE properly.\n";
          }
          continue;
        }
      }

      // load the plugin
      try
      {
        // Load the plugin into OGRE
        this->ogreRoot->loadPlugin(filename);
      }
      catch(Ogre::Exception &e)
      {
        if ((*piter).find("RenderSystem") != std::string::npos)
        {
          ignerr << "Unable to load Ogre Plugin[" << *piter
                 << "]. Rendering will not be possible."
                 << "Make sure you have installed OGRE properly.\n";
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::CreateRenderSystem()
{
  Ogre::RenderSystem *renderSys;
  const Ogre::RenderSystemList *rsList;

  rsList = &(this->ogreRoot->getAvailableRenderers());

  int c = 0;

  renderSys = nullptr;

  do
  {
    if (c == static_cast<int>(rsList->size()))
      break;

    renderSys = rsList->at(c);
    c++;
  }
  // cpplint has a false positive when extending a while call to multiple lines
  // (it thinks the while loop is empty), so we must put the whole while
  // statement on one line and add NOLINT at the end so that cpplint doesn't
  // complain about the line being too long
  while (renderSys && renderSys->getName().compare("OpenGL 3+ Rendering Subsystem") != 0); // NOLINT

  if (renderSys == nullptr)
  {
    ignerr << "unable to find OpenGL rendering system. OGRE is probably "
            "installed incorrectly. Double check the OGRE cmake output, "
            "and make sure OpenGL is enabled." << std::endl;
  }

  // We operate in windowed mode
  renderSys->setConfigOption("Full Screen", "No");

  renderSys->setConfigOption( "Interface", "Headless EGL / PBuffer" );

  {
    const Ogre::ConfigOptionMap &configOptions = renderSys->getConfigOptions();
    Ogre::ConfigOptionMap::const_iterator itor = configOptions.find( "Device" );
    if( itor == configOptions.end() )
    {
        fprintf( stderr, "Something must be wrong with EGL init. Cannot find Device" );
        return;
    }

    if( isatty( fileno( stdin ) ) )
    {
        printf( "Select device (this sample supports selecting between the first 10):\n" );

        int devNum = 0;
        Ogre::StringVector::const_iterator itDev = itor->second.possibleValues.begin();
        Ogre::StringVector::const_iterator enDev = itor->second.possibleValues.end();

        while( itDev != enDev )
        {
            printf( "[%i] %s\n", devNum, itDev->c_str() );
            ++devNum;
            ++itDev;
        }

        devNum = std::min( devNum, 10 );

        int devIdxChar = '0';//getchar();
        // while( devIdxChar < '0' || devIdxChar - '0' > devNum )
        //     devIdxChar = getchar();

        const uint32_t devIdx = static_cast<uint32_t>( devIdxChar - '0' );

        printf( "Selecting [%i] %s\n", devIdx, itor->second.possibleValues[devIdx].c_str() );
        renderSys->setConfigOption( "Device", itor->second.possibleValues[devIdx] );
    }
    else
    {
        printf( "!!! IMPORTANT !!!\n" );
        printf(
            "App is running from a file or pipe. Selecting a default device. Run from a real "
            "terminal for interactive selection\n" );
        printf( "!!! IMPORTANT !!!\n" );
        fflush( stdout );
    }
  }

  /// We used to allow the user to set the RTT mode to PBuffer, FBO, or Copy.
  ///   Copy is slow, and there doesn't seem to be a good reason to use it
  ///   PBuffer limits the size of the renderable area of the RTT to the
  ///           size of the first window created.
  ///   FBO seem to be the only good option
  // renderSys->setConfigOption("RTT Preferred Mode", "FBO");

  // get all supported fsaa values
  Ogre::ConfigOptionMap configMap = renderSys->getConfigOptions();
  auto fsaaOoption = configMap.find("FSAA");

  if (fsaaOoption != configMap.end())
  {
    auto values = (*fsaaOoption).second.possibleValues;
    for (auto const &str : values)
    {
      int value = 0;
      try
      {
        value = std::stoi(str);
      }
      catch(...)
      {
        continue;
      }
      this->dataPtr->fsaaLevels.push_back(value);
    }
  }
  std::sort(this->dataPtr->fsaaLevels.begin(), this->dataPtr->fsaaLevels.end());

  // check if target fsaa is supported
  unsigned int fsaa = 0;
  unsigned int targetFSAA = 4;
  auto const it = std::find(this->dataPtr->fsaaLevels.begin(),
      this->dataPtr->fsaaLevels.end(), targetFSAA);
  if (it != this->dataPtr->fsaaLevels.end())
    fsaa = targetFSAA;

  renderSys->setConfigOption("FSAA", std::to_string(fsaa));

  this->ogreRoot->setRenderSystem(renderSys);
}

void Ogre2RenderEngine::registerHlms()
{
  const char *env = std::getenv("IGN_RENDERING_RESOURCE_PATH");
  std::string resourcePath = (env) ? std::string(env) :
      IGN_RENDERING_RESOURCE_PATH;
  // install path
  std::string mediaPath = common::joinPaths(resourcePath, "ogre2", "media");
  if (!common::exists(mediaPath))
  {
    // src path
    mediaPath = common::joinPaths(resourcePath, "ogre2", "src", "media");
  }

  // register PbsMaterial resources
  Ogre::String rootHlmsFolder = mediaPath;
  Ogre::String pbsCompositorFolder = common::joinPaths(
      rootHlmsFolder, "2.0", "scripts", "Compositors");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      pbsCompositorFolder, "FileSystem", "Popular");
  Ogre::String commonMaterialFolder = common::joinPaths(
      rootHlmsFolder, "2.0", "scripts", "materials", "Common");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      commonMaterialFolder, "FileSystem", "General");
  Ogre::String commonGLSLMaterialFolder = common::joinPaths(
      rootHlmsFolder, "2.0", "scripts", "materials", "Common", "GLSL");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      commonGLSLMaterialFolder, "FileSystem", "General");
  Ogre::String commonGLSLESMaterialFolder = common::joinPaths(
      rootHlmsFolder, "2.0", "scripts", "materials", "Common", "GLSLES");
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      commonGLSLESMaterialFolder, "FileSystem", "General");

  // The following code is taken from the registerHlms() function in ogre2
  // samples framework
  if (rootHlmsFolder.empty())
    rootHlmsFolder = "./";
  else if (*(rootHlmsFolder.end() - 1) != '/')
    rootHlmsFolder += "/";

  // At this point rootHlmsFolder should be a valid path to the Hlms data folder

  // For retrieval of the paths to the different folders needed
  Ogre::String mainFolderPath;
  Ogre::StringVector libraryFoldersPaths;
  Ogre::StringVector::const_iterator libraryFolderPathIt;
  Ogre::StringVector::const_iterator libraryFolderPathEn;

  Ogre::ArchiveManager &archiveManager = Ogre::ArchiveManager::getSingleton();

  {
    Ogre::HlmsUnlit *hlmsUnlit = 0;
    // Create & Register HlmsUnlit
    // Get the path to all the subdirectories used by HlmsUnlit
    Ogre::HlmsUnlit::getDefaultPaths(mainFolderPath, libraryFoldersPaths);
    Ogre::Archive *archiveUnlit = archiveManager.load(
        rootHlmsFolder + mainFolderPath, "FileSystem", true);
    Ogre::ArchiveVec archiveUnlitLibraryFolders;
    libraryFolderPathIt = libraryFoldersPaths.begin();
    libraryFolderPathEn = libraryFoldersPaths.end();
    while (libraryFolderPathIt != libraryFolderPathEn)
    {
      Ogre::Archive *archiveLibrary =
          archiveManager.load(rootHlmsFolder + *libraryFolderPathIt,
          "FileSystem", true);
      archiveUnlitLibraryFolders.push_back(archiveLibrary);
      ++libraryFolderPathIt;
    }

    // Create and register the unlit Hlms
    hlmsUnlit = OGRE_NEW Ogre::HlmsUnlit(archiveUnlit,
        &archiveUnlitLibraryFolders);
    Ogre::Root::getSingleton().getHlmsManager()->registerHlms(hlmsUnlit);

    // disable writting debug output to disk
    hlmsUnlit->setDebugOutputPath(false, false);
  }

  {
    Ogre::HlmsPbs *hlmsPbs = 0;
    // Create & Register HlmsPbs
    // Do the same for HlmsPbs:
    Ogre::HlmsPbs::getDefaultPaths(mainFolderPath, libraryFoldersPaths);
    Ogre::Archive *archivePbs = archiveManager.load(
        rootHlmsFolder + mainFolderPath, "FileSystem", true);

    // Get the library archive(s)
    Ogre::ArchiveVec archivePbsLibraryFolders;
    libraryFolderPathIt = libraryFoldersPaths.begin();
    libraryFolderPathEn = libraryFoldersPaths.end();
    while (libraryFolderPathIt != libraryFolderPathEn)
    {
      Ogre::Archive *archiveLibrary =
          archiveManager.load(rootHlmsFolder + *libraryFolderPathIt,
          "FileSystem", true);
      archivePbsLibraryFolders.push_back(archiveLibrary);
      ++libraryFolderPathIt;
    }

    // Create and register
    hlmsPbs = OGRE_NEW Ogre::HlmsPbs(archivePbs, &archivePbsLibraryFolders);
    Ogre::Root::getSingleton().getHlmsManager()->registerHlms(hlmsPbs);

    // disable writting debug output to disk
    hlmsPbs->setDebugOutputPath(false, false);
  }
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::CreateResources()
{
  const char *env = std::getenv("IGN_RENDERING_RESOURCE_PATH");
  std::string resourcePath = (env) ? std::string(env) :
      IGN_RENDERING_RESOURCE_PATH;
  // install path
  std::string mediaPath = common::joinPaths(resourcePath, "ogre2", "media");
  if (!common::exists(mediaPath))
  {
    // src path
    mediaPath = common::joinPaths(resourcePath, "ogre2", "src", "media");
  }

  // register low level materials (ogre v1 materials)
  std::vector< std::pair<std::string, std::string> > archNames;
  std::string p = mediaPath;
  if (common::isDirectory(p))
  {
    archNames.push_back(
        std::make_pair(p, "General"));
    archNames.push_back(
        std::make_pair(p + "/materials/programs", "General"));
    archNames.push_back(
        std::make_pair(p + "/materials/scripts", "General"));
    archNames.push_back(
        std::make_pair(p + "/materials/textures", "General"));

    for (auto aiter = archNames.begin(); aiter != archNames.end(); ++aiter)
    {
      try
      {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            aiter->first, "FileSystem", aiter->second);
      }
      catch(Ogre::Exception &/*_e*/)
      {
        ignerr << "Unable to load Ogre Resources. Make sure the resources "
            "path in the world file is set correctly." << std::endl;
      }
    }
  }
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::CreateRenderWindow()
{
  // create dummy window
  auto res = this->CreateRenderWindow(std::to_string(this->dummyWindowId),
      800, 600, 1, 0);
  if (res.empty())
  {
    ignerr << "Failed to create dummy render window." << std::endl;
  }
}

//////////////////////////////////////////////////
std::string Ogre2RenderEngine::CreateRenderWindow(const std::string &_handle,
    const unsigned int _width, const unsigned int _height,
    const double _ratio, const unsigned int _antiAliasing)
{
  std::cerr << "/* Ogre2RenderEngine::CreateRenderWindow w: "<< _width << " "
    << _height << " useCurrentGLContext: " << useCurrentGLContext << '\n';
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  window = nullptr;

  // if use current gl then don't include window handle params
  if (!this->useCurrentGLContext)
  {
    // Mac and Windows *must* use externalWindow handle.
#if defined(__APPLE__) || defined(_MSC_VER)
    params["externalWindowHandle"] = _handle;
#else
    params["parentWindowHandle"] = _handle;
#endif
  }

  params["FSAA"] = std::to_string(_antiAliasing);
  params["stereoMode"] = "Frame Sequential";

  // TODO(anyone): determine api without qt

#if defined(__APPLE__)
  // Set the macAPI for Ogre based on the Qt implementation
  params["macAPI"] = "cocoa";
  params["macAPICocoaUseNSView"] = "true";
#endif

  // Hide window if dimensions are less than or equal to one.
  params["border"] = "none";

  std::ostringstream stream;
  stream << "OgreWindow(0)" << "_" << _handle;

  // Needed for retina displays
  params["contentScalingFactor"] = std::to_string(_ratio);

  // Ogre 2 PBS expects gamma correction
  params["gamma"] = "Yes";

  if (this->useCurrentGLContext)
  {
    params["externalGLControl"] = "true";
    params["currentGLContext"] = "true";
  }

  int attempts = 0;
  while (window == nullptr && (attempts++) < 10)
  {
    try
    {
      window = Ogre::Root::getSingleton().createRenderWindow(
          stream.str(), _width, _height, false, &params);
      this->registerHlms();
    }
    catch(const std::exception &_e)
    {
      ignerr << " Unable to create the rendering window: " << _e.what()
             << std::endl;
      window = nullptr;
    }
  }

  if (attempts >= 10)
  {
    ignerr << "Unable to create the rendering window after [" << attempts
           << "] attempts." << std::endl;
    return std::string();
  }

  if (window)
  {
    // window->setActive(true);
    window->_setVisible(true);

    // Windows needs to reposition the render window to 0,0.
    window->reposition(0, 0);
  }
  return stream.str();
}

//////////////////////////////////////////////////
void Ogre2RenderEngine::InitAttempt()
{
  this->initialized = false;

  // init the resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups(true);

  this->scenes = Ogre2SceneStorePtr(new Ogre2SceneStore);
}

/////////////////////////////////////////////////
std::vector<unsigned int> Ogre2RenderEngine::FSAALevels() const
{
  return this->dataPtr->fsaaLevels;
}

/////////////////////////////////////////////////
Ogre::v1::OverlaySystem *Ogre2RenderEngine::OverlaySystem() const
{
  return this->ogreOverlaySystem;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::rendering::Ogre2RenderEnginePlugin,
                    ignition::rendering::RenderEnginePlugin)
