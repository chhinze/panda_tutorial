
---
title: 'Guide: Get the Franka Emika Panda running with C++'
author: Christoph Hinze
date: \today
institute: IRTG Soft Tissue Robotics (GRK2198/1)
...

# Preliminaries:

1. The tutorial will run on Linux, explanations below are for Ubuntu. If you haven't already installed Ubuntu on your Computer, or enabled the Windows Sybsystem on Linux (WSL), you should install it, following the tutorial on <https://docs.microsoft.com/en-us/windows/wsl/install-win10>. As distribution install e.g. Ubuntu 18.04. This will enable you to run a virtual Ubuntu terminal on Windows 10.
2. Install Git on Windows (<https://git-scm.com/downloads>), or directly in the WSL (`sudo apt install git`)
3. Install requirements for building the C++ project that we will create throughout this tutorial.  We need `libfranka` (unfortunately not directly available through sources) and `libeigen3-dev`

    ```sh
    mkdir -p ~/dev && cd ~/dev
    libfranka_version="0.6.0"

    # install Eigen3 and libfranka 
    sudo apt update && sudo apt install --yes build-essential cmake git libpoco-dev libeigen3-dev

    # from here: build libfranka dependency from source code and install it:
    git clone --recursive https://github.com/frankaemika/libfranka.git
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

> The exercises without solution are directly available on the `master` branch. The solution is under branch `solution`. 

## Tutorial Project

Get the tutorial from Github with

```sh
# Drive C: on Windows (with WSL) is available with /mnt/c/..., so you might want to `cd` to this directory
git clone https://github.com/chhinze/panda_tutorial
```

## Ex. 1: Build the demo project and its documentation

Install the requirements for building the documentation

```sh
# --no-install-recommends needed to prevent installing doxygen-latex
sudo apt install --no-install-recommends doxygen graphviz
```


The project can be built with `cmake` with GNU `make`. I.e.

```sh
cd ~/dev/panda_tutorial
mkdir build 
cd build

# build the makefiles
cmake ..

# build the executables from C++ code
make -j4

# build the documentation with doxygen (needs doxygen installed)
make doc
```

The binaries are generated within the `./build/...`.
You can find the documentation at [`./build/documentation/html/index.html`](build/documentation/html/index.html)

## Ex. 2: Math operations with `Eigen3`

1. Create two `Eigen::Vector3d` objects
    - One with all entries 10 and 
    - one with [0.1, 0.5, 0.7];
2. Create two `Eigen::Matrix3d` objects:
    - An Identity matrix
    - A Matrix with
        ```
        | 1, 2, 3 |
        | 4, 5, 6 |
        | 7, 8, 9 |
        ```
3. Try, how to do vector-vector matrix-vector and matrix-matrix multiplications
4. Find out, how to perform element-wise operations. (E.g. find the row-wise maximum of a matrix, multiply element-wise).
5. Access the first two elements of a vector. Write new values to it.


## Ex. 3: Drive the robot on axes level

1. Have a look at the [MotionGenerator](https://frankaemika.github.io/libfranka/classMotionGenerator.html), which is originally provided by Franka Emika for their code samples. It is included as `"examples_common.h"`
2. Define a goal position as `std::array<double,7>` for the axes. The joint positions should be [0, -pi/4, 0, -3/4*pi, 0, pi/2, pi/4] (`0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4`). Create a MotionGenerator object with this goal position.
3. Set the robot controller to axis position control by just passing the `MotionGenerator` object. Refer to the [libfranka documentation](https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html) for different control modes.
4. Build and run the application (`make -j4 axes_motion`). Be careful, the robot will be moved. Be prepared to push the user stop button.

## Ex. 4: Drive the robot in a linear trajectory

1. Refer to the documentation to find out how to read a `franka::RobotState`. Read it once to get the sart position.
2. `O_T_EE_c` contains the homogeneous transformation matrix as array with 16 values. Convert it to a 6d vector (3 translations, 3 RPY rotations) by using the function `homogeneousTfArray2PoseVec(std::array<doublem 16>) -> Eigen::Vector6d`
3. Calculate a 6d goal pose by adding 0.1 m to each of the translational coordinates.
4. Create a `LinearTrajectory` between start an end pose. Use `v_max = 0.05`, `a_max = 0.5` and `j_max = 1e-3`.
5. Create a `TrajectoryIteratorCartesianVelocity` object. It overloads the function call operator, such that it can directly be used in `franka::Robot.control(...)`.
6. Specifiy the robot controller (`panda.control(...);`)
7. Build and run the application (`make -j4 cartesian_trajectory`). Be careful, the robot will be moved. Be prepared to push the user stop button.
