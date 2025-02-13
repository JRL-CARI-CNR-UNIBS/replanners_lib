#!/bin/bash

if [ -z "$PATH_TO_WS" ]; then
    mkdir -p openmore_ws/src
    mkdir -p openmore_ws/build
    mkdir -p openmore_ws/install
    cd openmore_ws
    export PATH_TO_WS="$(pwd)"

    echo "Workspace Path: $PATH_TO_WS"
    echo "PATH_TO_WS=$PATH_TO_WS" >> $GITHUB_ENV

    export PATH="$PATH_TO_WS/install/bin:$PATH"
    export LD_LIBRARY_PATH="$PATH_TO_WS/install/lib"
    export CMAKE_PREFIX_PATH="$PATH_TO_WS/install"

    echo "PATH: $PATH"
    echo "LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
    echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"

    echo "PATH=$PATH_TO_WS/install/bin:$PATH" >> "$GITHUB_ENV"
    echo "LD_LIBRARY_PATH=$PATH_TO_WS/install/lib" >> "$GITHUB_ENV"
    echo "CMAKE_PREFIX_PATH=$PATH_TO_WS/install" >> "$GITHUB_ENV"
fi

# Install graph_core
cd $PATH_TO_WS/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/graph_core.git

cd $PATH_TO_WS
mkdir -p build/graph_core
cmake -S src/graph_core/graph_core -B build/graph_core -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
make -C build/graph_core install

# Install replanners_lib
cd $PATH_TO_WS/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/replanners_lib.git

cd $PATH_TO_WS
mkdir -p build/replanners_lib
cmake -S src/replanners_lib -B build/replanners_lib -DCMAKE_INSTALL_PREFIX=$PATH_TO_WS/install
make -C build/replanners_lib install