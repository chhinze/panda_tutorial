
---
title: 'Guide: Get the Franka Emika Panda running with C++'
author: Christoph Hinze
date: \today
institute: IRTG Soft Tissue Robotics (GRK2198/1)
...

# Preliminaries:

1. The tutorial will run on Linux, explanations below are for Ubuntu. If you haven't already installed Ubuntu on your Computer, or enabled the Windows Sybsystem on Linux (WSL), you should install it, following the tutorial on <https://docs.microsoft.com/en-us/windows/wsl/install-win10>. As distribution install e.g. Ubuntu 18.04. This will enable you to run a virtual Ubuntu terminal on Windows 10.
2. Install Git on Windows (<https://docs.microsoft.com/en-us/windows/wsl/install-win10>), or directly in the WSL (`sudo apt install git`)
3. Install requirements for building the C++ project that we will create throughout this tutorial.  We need `libfranka` (unfortunately not directly available through sources) and `libeigen3-dev`

    ```sh
    mkdir -p ~/dev && cd ~/dev
    libfranka_version="0.6.0"

    # install Eigen3 and libfranka 
    sudo apt update && sudo apt install --yes build-essential cmake git libpoco-dev libeigen3-dev

    # from here: build libfranka dependency from source code and install it:
    git clone https://github.com/frankaemika/libfranka.git
    cd libfranka

    git checkout ${libfranka_version}
    git submodule update

    mkdir build && cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ..
    make -j4
    sudo make install
    ```
4. A manual on the Franka Emika Panda and `libfranka` is at <https://frankaemika.github.io/docs/getting_started.html>
5. A documentation for `libfranka` can be found at <https://frankaemika.github.io/libfranka/>
6. We will use `Eigen3` as math library, where an introduction may be found at <http://eigen.tuxfamily.org/dox/group__QuickRefPage.html>


# Theory

- [How the path planning is done](pathPlanning.ipynb)

# Exercises:

## Tutorial Project

get the tutorial from github with

```sh
cd ~/dev
git clone https://github.com/chhinze/panda_tutorial
```

## Ex. 1: Build the demo project and its documentation

## Ex. 2: Math operations with `Eigen3`

## Ex. 3: Drive the robot on axes level

## Ex. 4: Drive the robot in a linear trajectory