# smm_screws

Core screw theory library for Serial Metamorphic Manipulators (SMM) in ROS2.

## Overview

`smm_screws` implements screw-theory-based kinematics and dynamics for serial metamorphic manipulators.

The package is designed for ROS2, but can also be used as a standalone C++ library for robotics applications.

## Features

- Screw theory formulation (twists, adjoint transformations, exponentials)
- Analytical forward kinematics (stable)
- Partial dynamics implementation (under development)
- Support for 3-DOF and 6-DOF manipulators
- YAML-based robot structure loading
- Modular N-DOF implementation

## Important Notice

Use only the N-DOF implementation (`*Ndof.*` files).

Files without `Ndof`:
- are hardcoded for 3-DOF
- are deprecated
- will be removed in future versions

## Package Structure

## License

This project is licensed under the BSD 3-Clause License.
