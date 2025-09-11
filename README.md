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
* **`vizzy_robot`**: Contains launchers for the real Vizzy's hardware.
* **`vizzy_sensors`**: Contains filters to be applied to the real Vizzy's Hokuyo laser scanners.

## Architectural Note: Navigation Stack Dependency

A key decision in this project is the use of a specific development branch for the ROS 2 Navigation Stack (Nav2).

* **What we use:** Instead of the standard Nav2 version released with ROS 2 Humble, this project depends on the **`humble_main` branch** from the official [navigation2 repository](https://github.com/ros-navigation/navigation2/tree/humble_main).

* **Why we do this:** Our project is based on **ROS 2 Humble LTS** to ensure long-term stability. However, we also require advanced navigation features that were developed *after* Humble was released. By using the `humble_main` branch, we gain access to significant improvements backported from newer ROS 2 distributions. Key advantages include:
    * **New Safety Layers:** A dedicated `Collision Monitor` to prevent collisions independently of the controller, providing a last line of defense for safety. 🛡️
    * **Dedicated Docking Server:** A new `nav2_docking` package integrating the original [`opennav_docking`](https://github.com/open-navigation/opennav_docking) packages, which provide robust and standardized behaviors for docking at charging stations or specific poses.
    * **State-of-the-Art Controllers:** The MPPI Controller has been significantly optimized for speed (a reported 45% increase!), and its critics have been enhanced for more nuanced control.
    * **Streamlined Configuration:** The BT Navigator now features automated plugin management, eliminating the configuration of default plugins and simplifying the setup and configuration of custom behavior trees (dozens of configurations lines gone!).
    * **Modernized Internals:** Key libraries, like `BehaviorTree.CPP` (v4.5+), have been updated, providing better performance and access to new behavioral logic features.

    The `humble_main` branch allows us to leverage these modern capabilities while remaining on a stable ROS 2 distribution.

* **What this means for developers:** This approach provides access to cutting-edge features on a stable ROS 2 base. However, developers must be aware of the trade-offs: humble_main is an active development branch and may have temporary instabilities not present in the official release. Furthermore, our system's dependency on this specific Nav2 version may affect compatibility with external tools that expect a standard Humble installation. If you encounter any issues, please refer to our **Reporting Issues** section.

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

## Development and Recompiling

The `vizzy_install_2.0` script automatically compiles all packages in the selected colcon workspace during the initial setup. If you intend to modify the source code of these packages, you will need to recompile them for your changes to be applied.

The source code is typically located in `~/vizzy2_ws/src/` (adjust the path if you specified a different colcon workspace during installation).

**Steps to Recompile:**

1.  **Navigate to your Colcon Workspace Root:**
    Open a terminal and change to the root directory of your colcon workspace (e.g., `~/vizzy2_ws`).
    ```bash
    cd ~/vizzy2_ws 
    ```

2.  **Build the Workspace (or specific packages):**
    To rebuild all packages (a robust option that works in any use-case):
    ```bash
    colcon build --cmake-clean-cache
    ```
    * **`--cmake-clean-cache`**: Clears CMake's cache before building, forcing it to reconfigure. This is useful if you've modified `CMakeLists.txt` files, added/removed files, or want to ensure all changes are picked up.

    For potentially faster development cycles with certain types of changes (e.g., Python files, non-compiled resources), you can use `--symlink-install`:
    ```bash
    colcon build --symlink-install --cmake-clean-cache
    ```
    * **`--symlink-install`**: Creates symbolic links for some files instead of copying them into the `install` space. This allows some edits (e.g., to Python files) to take effect without a full rebuild. Be sure you understand its operation, as it can introduce complexities in some debugging scenarios if not handled carefully.

    To rebuild only specific packages you've modified (which is generally faster):
    ```bash
    colcon build --packages-select <package_name_1> <package_name_2> --cmake-clean-cache
    ```
    Or with symlink install:
    ```bash
    colcon build --symlink-install --packages-select <package_name_1> <package_name_2> --cmake-clean-cache
    ```

3.  **Source the Workspace Again (for the current terminal):**
    After a successful build, to make the newly compiled changes available in your *current* terminal session, re-source the workspace's setup file:
    ```bash
    source install/setup.bash
    ```
    *(New terminals will automatically pick up these changes.)*

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
