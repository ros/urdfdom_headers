# urdfdom_headers

[![License](https://img.shields.io/badge/License-BSD-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-2.1.0-brightgreen.svg)](package.xml)

[![License](https://img.shields.io/badge/License-BSD-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-2.1.0-brightgreen.svg)](package.xml)

`urdfdom_headers` provides the core C++ data structure headers for the Unified Robot Description Format (URDF). 

This repository is a **custom fork** of the standard `urdfdom_headers`. It adds specialized support for **constraint descriptions** to safely model closed-loop kinematics (e.g., for parallel robots) without breaking compatibility with the surrounding ROS ecosystem and packages.

## Features

- **Core URDF Headers**: Provides the same foundational C++ headers as the upstream `urdfdom_headers` package.
- **Closed-Loop Kinematics Support**: Introduces `constraint` elements representing "cut joints" or "cut links" to describe closed kinematic loops.
    - Based closely on the standard URDF `joint` element structure.
    - Adds a `child_frame` origin and a `child` axis to effectively describe the two frames of the cut element.
- **Constraint Types**: The explicit structure of the constraint is defined by giving the name (type) of the cut element:
  - `UNKNOWN`: Unknown constraint type
  - `REVOLUTE`: 1-DOF rotation joint constraint
  - `PRISMATIC`: Prismatic joint constraint
  - `UNIVERSAL`: Universal joint constraint
  - `SPHERICAL`: Spherical joint constraint
  - `LINK`: Link constraint
  - *(Additional types may be added in future updates)*

The description of constraints in this fork strictly adheres to standard modeling techniques specifically meant for parallel robots.

## Installation

### Prerequisites
- CMake (3.20 or higher)
- A C++ compiler

### Building from Source

```bash
mkdir build && cd build
cmake ..
make
sudo make install
```

## Usage

When used inside standard ROS build tools (`catkin`, `ament`, or `colcon` workspaces), the headers are installed in an extra subfolder (`include/urdfdom_headers/urdfdom_headers`) by default. This avoids include directory search order issues when overriding this package from a workspace.

To use this within your own CMake project:
```cmake
find_package(urdfdom_headers REQUIRED)
target_link_libraries(your_target PUBLIC urdfdom_headers::urdfdom_headers)
```

## Documentation

For general details about the standard URDF specifications and documentation on the basic elements, please refer to the [ROS URDF Wiki](http://ros.org/wiki/urdf).

## License

This project is licensed under the BSD License. See the [LICENSE](LICENSE) file for complete details.
