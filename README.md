# Vizzy Humanoid Robot - ROS 2 Core Packages

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange)](https://releases.ubuntu.com/22.04/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-blueviolet)](https://gazebosim.org/docs/fortress)

**🚧 Under Active Development & Migration 🚧**

**Please Note:** This repository contains the core packages for Vizzy, refactored for ROS 2 Humble. The migration from previous systems (ROS 1, YARP) is an ongoing process. While core simulation and basic functionalities are available, some features may still be under development or subject to change. We are actively working on completing the migration and will be posting regular updates. Your feedback and patience are appreciated!

Welcome to the core ROS 2 package suite for the Vizzy humanoid robot platform, refactored for **Ubuntu 22.04** and **ROS 2 Humble Hawksbill**. This repository provides essential software for simulation, control, perception, and interaction with Vizzy using the ROS 2 framework and Ignition Gazebo Fortress.

For automated system setup and installation of all dependencies, please use our dedicated installer: [**vizzy_install_2.0**](https://github.com/vislab-tecnico-lisboa/vizzy_install_2.0).

![vizzy_grasping_ball](https://github.com/user-attachments/assets/b8942c91-66ad-4de3-8e03-234f22d924b6)

## Overview

This repository contains a focused set of ROS 2 packages necessary to simulate and interact with the Vizzy robot. The primary goal is to provide a streamlined experience for users working with Vizzy in a ROS 2 Humble environment. All YARP dependencies and older ROS 1 specific code from previous versions have been deprecated or not yet migrated.

## Packages

This repository currently includes the following core ROS 2 packages:

* **`vizzy_launch`**: Contains primary launch files to bring up the Vizzy simulation, navigation stack, and other core functionalities. For most users, this is the main entry point.
* **`vizzy_description`**: Holds Vizzy's robot description files (URDF, meshes, kinematics, visual data, etc.).
* **`vizzy_gazebo`**: Provides launch files and configurations required to simulate Vizzy in the Ignition Gazebo Fortress environment.
* **`vizzy_navigation`**: Contains launch files and configurations for deploying the ROS 2 Navigation Stack (Nav2) with Vizzy.
* **`vizzy_msgs`**: Defines custom ROS 2 messages, services, and actions specific to Vizzy.

## Prerequisites

To use these packages, your system should meet the following requirements:

* **Operating System:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS 2 Version:** Humble Hawksbill (Desktop Install)
* **Gazebo Version:** Ignition Gazebo Fortress
* **Essential Tools:** `git`, `colcon` (ROS 2 build tool), `rosdep`

**Note:** For your convenience, all listed prerequisites are managed and installed by our dedicated installer.

## Installation and Setup

The recommended method for setting up your system and installing these Vizzy packages is by using the scripts provided in the [**vizzy_install_2.0**](https://github.com/vislab-tecnico-lisboa/vizzy_install_2.0) repository.

The installer script will:
1.  Guide you through system dependencies installation (ROS 2 Humble, Gazebo Fortress, etc.).
2.  Ask for your preferences (e.g., NVIDIA GPU usage for Gazebo, colcon workspace location).
3.  Clone this `vizzy2.0` repository (containing these ROS 2 packages) into the specified colcon workspace (default: `~/vizzy2_ws/src/`).
4.  Use `rosdep` to install package dependencies.
5.  Build the colcon workspace.
6.  Configure your `.bashrc` to source ROS 2 and the Vizzy workspace.

**Please refer to the `README.md` in the [vizzy_install_2.0 repository](https://github.com/vislab-tecnico-lisboa/vizzy_install_2.0) for detailed installation instructions.**

After running the installer and sourcing your `~/.bashrc` (or opening a new terminal), your environment should be ready.

## Running the Simulation

Once your environment is set up using the `vizzy_install_2.0` scripts:

1.  **Ensure your shell environment is updated:**
    If you haven't already, open a new terminal or run:
    ```bash
    source ~/.bashrc
    ```

2.  **Launch the main Vizzy simulation:**
    This typically includes Gazebo, Nav2, and RViz2.
    ```bash
    ros2 launch vizzy_launch vizzy_simulation_launch.xml
    ```
    * **NVIDIA GPU Offloading:** If you opted for NVIDIA GPU usage during the installation via `vizzy_install_2.0`, the launch file should automatically attempt to use your NVIDIA GPU.
    * The `vizzy_install_2.0` script also provides an alias `ign-nvidia` for manual Gazebo launching with NVIDIA offloading if needed for non-ROS 2 launch scenarios.

## Documentation & Citation

For more details on the Vizzy platform, please refer to the following publication:

```bibtex
@inproceedings{moreno2016vizzy,
  title={Vizzy: A humanoid on wheels for assistive robotics},
  author={Moreno, Plinio and Nunes, Ricardo and Figueiredo, Rui and Ferreira, Ricardo and Bernardino, Alexandre and Santos-Victor, Jos{\'e} and Beira, Ricardo and Vargas, Lu{\'\i}s and Arag{\~a}o, Duarte and Arag{\~a}o, Miguel},
  booktitle={Robot 2015: Second Iberian Robotics Conference},
  pages={17--28},
  year={2016},
  organization={Springer}
}
```

## Reporting Issues

Please report any bugs, issues, or feature requests related to these ROS 2 packages using the [GitHub Issues tab](https://github.com/vislab-tecnico-lisboa/vizzy2.0/issues) of this repository.

For issues related to the system installation process, please use the issue tracker of the [vizzy_install_2.0 repository](https://github.com/vislab-tecnico-lisboa/vizzy_install_2.0/issues).
