# Vizzy Humanoid Robot - ROS 2 Core Packages

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20LTS-orange)](https://releases.ubuntu.com/22.04/)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-blueviolet)](https://gazebosim.org/docs/fortress)
[![Nav2 Version](https://img.shields.io/badge/Nav2-humble__main-yellowgreen)](https://github.com/ros-navigation/navigation2/tree/humble_main)

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
  
## 🛠️ Simulation Configuration Parameters

The Vizzy simulation can be configured using a wide range of launch arguments, allowing users to tailor robot behavior, environment setup, sensor topics, navigation stack settings, and docking behavior. Below is a complete list of the configurable parameters, their defaults, and brief descriptions.

### Main Simulation Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `base_frame_id` | `base_footprint` | Base frame ID of the robot. |
| `odom_frame_id` | `odometry` | Odometry frame used in the transform tree. |
| `odom_topic` | `/odom` | Topic name where odometry messages are published. |
| `map_frame_id` | `map` | Map frame used in the transform tree. |
| `pose` | `-x 0.0 -y 0.0 -z 0.0 -Y 0.0` | Initial pose of the robot in the simulation. |
| `urdf_file` | `vizzy.urdf.xacro` | URDF/XACRO file used to describe the robot model. |
| `paused` | `false` | Whether the simulation starts in a paused state. |
| `map_topic` | `/map` | Topic name for the global map. |
| `map_file` | `isr_7th_floor_simulation.yaml` | Main 2D map file (YAML format). |
| `obstacles_map_file` | `mapa_piso7_NOVO_AWESOME_obst.yaml` | Obstacle map file (YAML format). |
| `world` | `isr_7th_floor_world.sdf` | Ignition Gazebo world file. |
| `use_obstacles` | `false` | Whether to load and use the obstacles map. |
| `log_level` | `info` | ROS 2 logging level. |
| `scan_topic_front` | `/nav_hokuyo_laser/front/scan` | Topic name for the front laser scanner. |
| `scan_topic_rear` | `/nav_hokuyo_laser/rear/scan` | Topic name for the rear laser scanner. |
| `controller_frequency` | `20.0` | Frequency at which the controller runs (Hz). |
| `path_align_forward_point_distance` | `0.325` | Forward point distance for the PathAlign critic. |
| `goal_align_forward_point_distance` | `0.325` | Forward point distance for the GoalAlign critic. |
| `inflation_radius` | `0.50` | Radius used for inflation in the costmap. |
| `path_align_scale` | `32.0` | Scaling factor for the PathAlign critic. |
| `goal_align_scale` | `24.0` | Scaling factor for the GoalAlign critic. |
| `path_dist_scale` | `32.0` | Scaling factor for the PathDist critic. |
| `goal_dist_scale` | `24.0` | Scaling factor for the GoalDist critic. |
| `cost_scaling_factor` | `3.0` | Scaling factor for costmap inflation layer. |
| `base_obstacle_scale` | `0.02` | Scaling factor for base obstacles in the costmap. |
| `controller_plugin_type` | `MPPI` | Controller plugin to use: `DWB`, `RPP`, or `MPPI`. |
| `expected_planner_frequency` | `0.1` | Expected planner frequency (Hz). |
| `planner_plugin_type` | `ThetaStar` | Planner plugin to use (e.g., `NavFn`, `Smac`, `ThetaStar`). |
| `robot_radius` | `0.4` | Robot's radius used by the costmap and collision logic. |

### MPPI Controller Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `mppi_cost_critic_cost_weight` | `3.82` | General cost weight for MPPI cost critic. |
| `mppi_path_align_critic_cost_weight` | `14.0` | General cost weight for MPPI path alignment critic. |
| `mppi_wz_std` | `0.2` | Standard deviation for MPPI angular velocity sampling. |
| `mppi_wide_cost_critic_cost_weight` | `3.82` | Cost weight for wide MPPI cost critic. |
| `mppi_narrow_cost_critic_cost_weight` | `10.0` | Cost weight for narrow MPPI cost critic. |
| `mppi_wide_path_align_critic_cost_weight` | `14.0` | Path align weight for wide MPPI critic. |
| `mppi_narrow_path_align_critic_cost_weight` | `32.0` | Path align weight for narrow MPPI critic. |
| `mppi_obstacles_critic_repulsion_weight` | `1.5` | Repulsion weight for MPPI obstacles critic. |
| `mppi_obstacles_critic_critical_weight` | `20.0` | Critical penalty for MPPI obstacles critic. |
| `mppi_obstacles_critic_collision_margin_distance` | `0.2` | Collision margin distance for MPPI obstacles. |
| `mppi_wide_wz_std` | `0.2` | Standard deviation for wide MPPI controller. |
| `mppi_narrow_wz_std` | `0.13` | Standard deviation for narrow MPPI controller. |

### Docking & Battery Simulation Parameters

| Parameter Name | Default Value | Description |
|----------------|---------------|-------------|
| `velocity_smoother_feedback_type` | `OPEN_LOOP` | Velocity feedback mode: `OPEN_LOOP` or `CLOSED_LOOP`. |
| `battery_state` | `0` | Initial battery state: `0` = not charging, `1` = charging. |
| `use_battery_state_simulation` | `true` | Enables simulated battery behavior. |
| `publish_dock_point_cloud` | `false` | Whether to publish the dock point cloud. |
| `activate_dock_pose_detection` | `false` | Whether to activate docking pose detection on startup. |

---

These parameters can be overridden directly from the launch command:

```bash
ros2 launch vizzy_launch vizzy_simulation_launch.xml use_obstacles:=true inflation_radius:=1.0
```

Adjusting these values allows you to customize simulation performance, navigation behavior, and controller/docking behavior to fit your use case or experiment.

## Setup the Real Vizzy (Developer & Authorized-Only Guide)
For those to whom this may concern, we leave here a setup tutorial for the real Vizzy with detailed steps.

1.  **Update the System**
    Ensure your system's package list and installed packages are up to date.
    ```bash
    sudo apt update
    sudo apt upgrade -y
    ```

2.  **Clone Necessary Segway Driver Packages**
    * Clone the serial folder in parallel to Vizzy's workspace source folder.
    ```bash
    cd ~/vizzy2_ws/src
    git clone https://github.com/utexas-bwi/serial_for_ros2.git
    ```

    * Clone the libsegwayrmp_ros2 library as well.
    ```bash
    git clone https://github.com/utexas-bwi/libsegwayrmp_ros2.git
    ```
    
3.  **Clone the Navigation2 humble_main branch**
    * Navigate into the vizzy2 workspace's source folder and clone the official navigation2 backported humble version.
    ```bash
    cd ~/vizzy2/vizzy2_ws/src/
    git clone https://github.com/ros-navigation/navigation2.git --branch humble_main
    ```
4.  **Clone and Build the TEASER++ Repository**
      ```bash
      git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
      cd TEASER-plusplus
      mkdir build
      cd build
      cmake ..
      make -j$(nproc)
      sudo make install
      ```

4.  **Install the ROS2 Dependencies**
    Use rosdep to install the necessary dependencies while disregarding dev packages.
    ```bash
    cd ..
    rosdep install -i --from-path src --rosdistro humble -y --skip-keys "nav2_minimal_tb3_sim nav2_minimal_tb4_sim"
    ```
5.  **Remove Apt-Installed Dependencies**
    Sometimes rosdep installs dependencies from packages that are present in the source folder (ignoring the -i command). For this,
     we need to manually remove any installed nav2 dependence.
    ```bash
    sudo apt remove "ros-humble-nav2-*"
    rm -rf build/ install/ log/
    ```
6.  **Tell Colcon to Ignore Certain Packages**
    Because we want to focus just on nav2 now, let us ignore all the other packages.
    ```bash
    touch src/navigation2/nav2_system_tests/COLCON_IGNORE
    touch src/vizzy2/COLCON_IGNORE
    touch src/navigation2/segway_rmp_ros2/COLCON_IGNORE
    touch src/navigation2/libsegway_rmp_ros2/COLCON_IGNORE
    ```
7.  **Build the Package**
    Let us build everything.
    ```bash
    colcon build --symlink-install
    ```
8.  **Final Details**
    Once (and if) everything is correctly built, we can safely ignore the nav2 and segway drivers packages in future builds and remove the ignore command on our main vizzy2 folder.
    ```bash
    rm src/vizzy2/COLCON_IGNORE
    touch src/navigation2/COLCON_IGNORE
    ouch src/libsegwayrmp_ros2/COLCON_IGNORE
    ouch src/segway_rmp_ros2/COLCON_IGNORE
    ```

For more information regarding this version of nav2, please refer to [their official repository](https://github.com/ros-navigation/navigation2/tree/humble_main).

At the end of the setup, you should have something that looks exactly like this:

<img width="615" height="172" alt="Screenshot from 2025-09-23 07-01-26" src="https://github.com/user-attachments/assets/fa38f9b9-2063-4d99-8bc8-8692d848b928" />

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
