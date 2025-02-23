# **replanners_lib**

[![build](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/actions/workflows/build_and_install.yaml/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/actions/workflows/build_and_install.yaml)
[![clang-format](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/actions/workflows/clang-format.yaml/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/actions/workflows/clang-format.yaml)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/120098816ece42c4a8387a2ca6501f25)](https://app.codacy.com/gh/JRL-CARI-CNR-UNIBS/replanners_lib/dashboard?utm_source=gh&utm_medium=referral&utm_content=&utm_campaign=Badge_grade)
![Status](https://img.shields.io/badge/License-BSD3-green)


`replanners_lib` is a C++ library that offers a suite of sampling-based path replanning algorithms. These algorithms are designed to rapidly adjust or modify the robot's current path, responding to dynamic environmental changes within tens to few hundreds of milliseconds. Unlike traditional path planning algorithms that compute paths from scratch and require longer computation times, path replanning leverages prior search information to significantly accelerate the process of finding a new, valid path.

This library is part of the [`OpenMORE`](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE.git) project, which provides a comprehensive set of tools to develop and execute sampling-based path replanning algorithms during robot motion.

`replanners_lib` relies on [`graph_core`](https://github.com/JRL-CARI-CNR-UNIBS/graph_core) for foundational classes essential to sampling-based path planning.

---
This is the list of the currently implemented replanners:

1. [MARS](https://ieeexplore.ieee.org/document/10013661?source=authoralert)
2. [DRRT](https://ieeexplore.ieee.org/document/1641879)
3. [Anytime DRRT](https://ieeexplore.ieee.org/document/4209270)
4. [DRRT*](https://ieeexplore.ieee.org/document/8122814)
5. [MPRRT](https://ieeexplore.ieee.org/document/7027233)

## Build & Install
Download dependencies using [vcstool](https://github.com/dirk-thomas/vcstool) and the [`deps.repos` file](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/deps.repos). Within your workspace, do:
```bash
mkdir -p openmore_ws/src && cd openmore_ws/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib.git
vcs import < replanners_lib/deps.repos
cd  ..
```

Then, from within the workspace follow this instructions to build:

1. Setup the environment and build
```bash
mkdir -p build/replanners_lib
cmake -S replanners_lib -B build/replanners_lib -DCMAKE_INSTALL_PREFIX=${HOME}/openmore_ws/install
make -C build/replanners_lib install
```

2. Add the path to the install folder in the `.bashrc` file so that other packages will be able to find it:

```bash
if [[ ":$PATH:" != *":$HOME/openmore_ws/install/bin:"* ]]; then
    export PATH="$HOME/openmore_ws/install/bin:$PATH"
fi
if [[ ":$LD_LIBRARY_PATH:" != *":$HOME/openmore_ws/install/lib:"* ]]; then
    export LD_LIBRARY_PATH="$HOME/openmore_ws/install/lib:$LD_LIBRARY_PATH"
fi
if [[ ":$CMAKE_PREFIX_PATH:" != *":$HOME/openmore_ws/install:"* ]]; then
    export CMAKE_PREFIX_PATH="$HOME/openmore_ws/install:$CMAKE_PREFIX_PATH"
fi
```

You can also build `replanners_lib` within a Catkin workspace, but before set `catkin config --install`.


## ROS Compatibility

While `replanners_lib` is ROS-free, it can utilize [`graph_display`](https://github.com/JRL-CARI-CNR-UNIBS/graph_display.git) for debugging if ROS is available. `graph_display` provides functionalities to display paths, trees, etc., in Rviz and is integrated into some replanners for debugging purposes. If ROS and `graph_display` are available and sourced this opion is ON by default, but you can disable it by setting `-DUSE_GRAPH_DISPLAY=False`

If you are using ROS 1 and want to include `replanners_lib` into a ROS package, download it in a catkin workspace and set `catkin config --install` to make other ROS packages able to find it.

## Tutorials
Visit [this](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/blob/master/documentation/tutorial/tutorial.md) page for tutorials on how you can use a replanner or implement a new algorithm using `replanners_lib`.
