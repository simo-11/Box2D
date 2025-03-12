<<<<<<< HEAD
![Box2D Logo](https://box2d.org/images/logo.svg)

# Build Status
[![Build Status](https://github.com/erincatto/box2d/actions/workflows/build.yml/badge.svg)](https://github.com/erincatto/box2d/actions)

# Box2D 

Box2D is a 2D physics engine for games.

## Contributing

Please do not submit pull requests with new features or core library changes. Instead, please file an issue first for discussion. For bugs, I prefer detailed bug reports over pull requests.

## Features

### Collision
- Continuous collision detection
- Contact callbacks: begin, end, pre-solve, post-solve
- Convex polygons and circles
- Multiple shapes per body
- One-shot contact manifolds
- Dynamic tree broadphase
- Efficient pair management
- Fast broadphase AABB queries
- Collision groups and categories

### Physics
- Continuous physics with time of impact solver
- Persistent body-joint-contact graph
- Island solution and sleep management
- Contact, friction, and restitution
- Stable stacking with a linear-time solver
- Revolute, prismatic, distance, pulley, gear, mouse joint, and other joint types
- Joint limits, motors, and friction
- Momentum decoupled position correction
- Fairly accurate reaction forces/impulses

### System
- Small block and stack allocators
- Centralized tuning parameters
- Highly portable C++ with no use of STL containers

### Testbed
- OpenGL with GLFW
- Graphical user interface with imgui
- Extensible test framework
- Support for loading world dumps

## Building
- Install [CMake](https://cmake.org/)
- Ensure CMake is in the user `PATH`
- Visual Studio: run `build.bat` from the command prompt
- Otherwise: run `build.sh` from a bash shell
- Results are in the build sub-folder
- On Windows you can open box2d.sln

## Building Box2D - Using vcpkg
You can download and install Box2D using the [vcpkg](https://github.com/Microsoft/vcpkg) dependency manager:

- git clone https://github.com/Microsoft/vcpkg.git
- cd vcpkg
- ./bootstrap-vcpkg.sh
- ./vcpkg integrate install
- ./vcpkg install box2d

The Box2D port in vcpkg is kept up to date by Microsoft team members and community contributors. If the version is out of date, please [create an issue or pull request](https://github.com/Microsoft/vcpkg) on the vcpkg repository.

Note: vcpkg support is not provided by the Box2D project

## Building for Xcode
- Install [CMake](https://cmake.org)
- Add Cmake to the path in .zprofile (the default Terminal shell is zsh)
    - export PATH="/Applications/CMake.app/Contents/bin:$PATH"
- mkdir build
- cd build
- cmake -G Xcode ..
- open box2d.xcodeproj
- Select the testbed scheme
- Edit the scheme to set a custom working directory, make this be in box2d/testbed
- You can now build and run the testbed

## Installing using CMake
You can build and install the library and docs using this command sequence (requires Doxygen):
```
mkdir build
cd build
cmake -DBOX2D_BUILD_DOCS=ON ..
cmake --build .
cmake --build . --target INSTALL
```
On Windows this tries to install in `Program Files` and thus requires admin privileges. Alternatively you can target another directory using something like this:
```
mkdir build
cd build
cmake -DBOX2D_BUILD_DOCS=ON -DCMAKE_INSTALL_PREFIX="C:/packages" ..
cmake --build .
cmake --build . --target INSTALL
```

## Documentation
- [Manual](https://box2d.org/documentation/)
- [reddit](https://www.reddit.com/r/box2d/)
- [Discord](https://discord.gg/NKYgCBP)

## License
Box2D is developed by Erin Catto, and uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License).

## Sponsorship
Support development of Box2D through [Github Sponsors](https://github.com/sponsors/erincatto)
=======
![Box2D Logo](https://box2d.org/images/logo.svg)

# Build Status
[![Build Status](https://github.com/erincatto/box2d/actions/workflows/build.yml/badge.svg)](https://github.com/erincatto/box2d/actions)

# Box2D 
Box2D is a 2D physics engine for games.

[![Box2D Version 3.0 Release Demo](https://img.youtube.com/vi/dAoM-xjOWtA/0.jpg)](https://www.youtube.com/watch?v=dAoM-xjOWtA)

## Features

### Collision
- Continuous collision detection
- Contact events and sensors
- Convex polygons, capsules, circles, rounded polygons, segments, and chains
- Multiple shapes per body
- Collision filtering
- Ray casts, shape casts, and overlap queries

### Physics
- Robust _Soft Step_ rigid body solver
- Continuous physics for fast translations and rotations
- Island based sleep
- Revolute, prismatic, distance, mouse joint, weld, and wheel joints
- Joint limits, motors, springs, and friction
- Joint and contact forces
- Body movement events and sleep notification

### System
- Data-oriented design
- Written in portable C17
- Extensive multithreading and SIMD

### Samples
- OpenGL with GLFW and enkiTS
- Graphical user interface with imgui
- Many samples to demonstrate features and performance

## Building
- Install [CMake](https://cmake.org/)
- Ensure CMake is in the user `PATH`
- Visual Studio: run `build.bat` from the command prompt
- Otherwise: run `build.sh` from a bash shell
- Results are in the build sub-folder
- On Windows you can open box2d.sln

## Building for Xcode
- Install [CMake](https://cmake.org)
- Add Cmake to the path in .zprofile (the default Terminal shell is zsh)
    - export PATH="/Applications/CMake.app/Contents/bin:$PATH"
- mkdir build
- cd build
- cmake -G Xcode ..
- open box2d.xcodeproj
- Select the samples scheme
- Edit the scheme to set a custom working directory to the box2d directory
- You can now build and run the samples

## Compatibility
The Box2D library and samples build and run on Windows, Linux, and Mac.

Box2D should be built on recent versions of clang and gcc. You will need the latest Visual Studio version for C11 atomics to compile (17.8.3+).

## Documentation
- [Manual](https://box2d.org/documentation/)
- [Migration Guide](https://github.com/erincatto/box2d/blob/main/docs/migration.md)

## Community
- [Discord](https://discord.gg/NKYgCBP)

## Contributing
Please do not submit pull requests. Instead, please file an issue for bugs or feature requests. For support, please visit the Discord server.

# Giving Feedback
Please file an issue or start a chat on discord.

## License
Box2D is developed by Erin Catto and uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License).

## Sponsorship
Support development of Box2D through [Github Sponsors](https://github.com/sponsors/erincatto).

Please consider starring this repository and subscribing to my [YouTube channel](https://www.youtube.com/@erin_catto).

## Ports, wrappers, and bindings
- Beef bindings - https://github.com/EnokViking/Box2DBeef
- C++ bindings - https://github.com/HolyBlackCat/box2cpp
- WASM - https://github.com/Birch-san/box2d3-wasm
