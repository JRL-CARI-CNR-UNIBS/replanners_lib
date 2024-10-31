# **replanners_lib**

The **replanners_lib** package provides a collection of sample-based path re-planning algorithms.
It is part of [*OpenMORE*](https://github.com/JRL-CARI-CNR-UNIBS/OpenMORE.git), a broader repository which provides tools to use and develop new sampling-based path re-planning algorithms during robot's motion. 

This is the list of the currently implemented replanners:

1. [MARS](https://ieeexplore.ieee.org/document/10013661?source=authoralert)
2. [DRRT](https://ieeexplore.ieee.org/document/1641879)
3. [Anytime DRRT](https://ieeexplore.ieee.org/document/4209270)
4. [DRRT*](https://ieeexplore.ieee.org/document/8122814)
5. [MPRRT](https://ieeexplore.ieee.org/document/7027233)

*replanners_lib* depends on [*graph_core*](https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git), which provides the necessary classes and solvers for path planning problems.

## Installation

To install *replanners_lib*, follow these steps:

```bash
mkdir -p openmore_ws/build/replanners_lib && cd openmore_ws
cmake -S replanners_lib -B build/replanners_lib -DCMAKE_INSTALL_PREFIX=${HOME}/openmore_ws/install
make -C build/replanners_lib install
```

Then add the path to the install folder in the `.bashrc` file so that other packages will be able to find it:

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

## ROS Compatibility

While *replanners_lib* is ROS-free, it can utilize [*graph_display*](https://github.com/JRL-CARI-CNR-UNIBS/graph_display.git) for debugging if ROS is available. *graph_display* provides functionalities to display paths, trees, etc., in Rviz and is integrated into some replanners for debugging purposes. If ROS and *graph_display* are available and sourced this opion is ON by default, but you can disable it by setting `-DUSE_GRAPH_DISPLAY=False`

If you are using ROS 1 and want to include *replanners_lib* into a ROS package, download it in a catkin workspace and set `catkin config --install` to make other ROS packages able to find it.

## How to use
[Here](https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib/blob/master/documentation/tutorial.md) a quick tutorial on how you can use a replanner or implement your own algorithm.

## Documentation Update

<h1 align="center">🚧 Update in Progress! 🚧</h1>
<p align="center">
  <img src="https://img.shields.io/badge/Status-Updating-blue?style=for-the-badge&logo=github">
</p>
<p align="center">
  We're currently working on documentation. Expect new changes in the next few weeks. Stay tuned!
</p>
