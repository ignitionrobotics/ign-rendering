# Ignition Rendering: Rendering library for robot applications

**Maintainer:** ichen [AT] openrobotics [DOT] org

[![GitHub open issues](https://img.shields.io/github/issues-raw/ignitionrobotics/ign-rendering.svg)](https://github.com/ignitionrobotics/ign-rendering/issues)
[![GitHub open pull requests](https://img.shields.io/github/issues-pr-raw/ignitionrobotics/ign-rendering.svg)](https://github.com/ignitionrobotics/ign-rendering/pulls)
[![Discourse topics](https://img.shields.io/discourse/https/community.gazebosim.org/topics.svg)](https://community.gazebosim.org)
[![Hex.pm](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0)

Build | Status
-- | --
Test coverage | [![codecov](https://codecov.io/gh/ignitionrobotics/ign-rendering/branch/ign-rendering4/graph/badge.svg)](https://codecov.io/gh/ignitionrobotics/ign-rendering/branch/default)
Ubuntu Bionic | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_rendering-ci-ign-rendering4-bionic-amd64)](https://build.osrfoundation.org/job/ignition_rendering-ci-ign-rendering4-bionic-amd64)
Homebrew      | [![Build Status](https://build.osrfoundation.org/buildStatus/icon?job=ignition_rendering-ci-ign-rendering4-homebrew-amd64)](https://build.osrfoundation.org/job/ignition_rendering-ci-ign-rendering4-homebrew-amd64)
Windows       | [![Build Status](https://build.osrfoundation.org/job/ign_rendering-ign-4-win/badge/icon)](https://build.osrfoundation.org/job/ign_rendering-ign-4-win/)

Ignition Rendering is a C++ library designed to provide an abstraction
for different rendering engines. It offers unified APIs for creating
3D graphics applications.

Ignition Rendering is a component in the ignition framework, a set
of libraries designed to rapidly develop robot applications.

# Table of Contents

[Features](#features)

[Install](#install)

[Usage](#usage)

[Folder Structure](#folder-structure)

[Code of Conduct](#code-of-conduct)

[Contributing](#code-of-contributing)

[Versioning](#versioning)

[License](#license)

# Features

* Support for rendering engines including OGRE and OptiX
* Plugin-based architecture. Mulitple rendering engine plugins can be loaded at run time.
* Object-oriented scene management.

# Install

See the [installation tutorial](https://ignitionrobotics.org/api/rendering/4.0/installation.html).

# Usage

The Ign Rendering API can be found in the documentation. See the
[installation tutorial](https://ignitionrobotics.org/api/rendering/4.0/installation.html)
on how to build the documentation files using Doxygen.

You can also take a look at the sample applications in the `examples` folder.

# Folder Structure

* `include/ignition/rendering`: Contains all the public header files which will be installed

* `src`: Contains all the C++ source code which are not installed.

* `test`: All integration, performance and regression tests go here, under their
  specific folders.

* `examples`: Sample programs to demonstrate different features of ign-rendering

* `doc`: Files used by Doxygen when generating documentation.

Rendering engine plugin implementation code is stored in their own folders

* `ogre` : OGRE 1.x rendering engine plugin

* `ogre2` : OGRE 2.x rendering engine plugin (available in versions >= ign-rendering1)

* `optix` : OptiX rendering engine plugin

# Contributing

Please see
[CONTRIBUTING.md](https://ignitionrobotics.org/docs/all/contributing).

# Code of Conduct

Please see
[CODE_OF_CONDUCT.md](https://github.com/ignitionrobotics/ign-gazebo/blob/main/CODE_OF_CONDUCT.md).

# Versioning

This library uses [Semantic Versioning](https://semver.org/). Additionally, this library is part of the [Ignition Robotics project](https://ignitionrobotics.org) which periodically releases a versioned set of compatible and complimentary libraries. See the [Ignition Robotics website](https://ignitionrobotics.org) for version and release information.

# License

This library is licensed under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0). See also the [LICENSE](https://github.com/ignitionrobotics/ign-rendering/blob/main/LICENSE) file.
