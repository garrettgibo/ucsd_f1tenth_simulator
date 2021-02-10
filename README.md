# UCSD F1/10 Gazebo Simulator

This simulator is meant to mimic the functionality and usage of the f1tenth framework as best as possible while using the UCSD Warren East Track.

* NOTE: This has only been tested on a system of the following specs

|     | Specifications              |
|:---:|:----------------------------|
| OS  | Ubuntu 20.04 (Focal) 64-bit |
| ROS | Noetic                      |

## Installation

### 1. ROS Noetic and Gazebo

This simulation was tested and developed on Ubuntu 20.04 with ROS Noetic.
Please follow [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)
instructions to install ROS.

### 2. Catkin Tools

`catkin_tools` is the preferred build tool; however, as of the time of this
creation, the default installation with `pip` was having some problems, so
a source installation is necessary.

```sh
$ pip3 install git+https://github.com/catkin/catkin_tools.git
```

### 3. Workspace Setup

#### 3.1 Create the root of the workspace

```sh
mkdir -p ~/ucsd_sim_ws/src
cd ~/ucds_sim_ws
```

#### 3.2 Configure workspace settings

```sh
catkin config --extend /opt/ros/noetic
```

### 4. Simulation Setup

#### 4.1 Clone this simulation repo

```sh
cd src
git clone https://github.com/garrettgibo/ucsd_f1tenth_system
cd ucsd_f1tenth_system
```

#### 4.2 Install Intel RealSense Gazebo Plugin

The default Intel RealSense Package does not natively support integration with
gazebo, so we are using this package: [https://github.com/SyrianSpock/realsense_gazebo_plugin](https://github.com/SyrianSpock/realsense_gazebo_plugin)

This will be setup with rosinstall

```sh
rosinstall .
```

### 5. Build Simuation

Build the ROS packages using the standard methods with `catkin_tools`

```sh
cd ~/ucsd_sim_ws
catkin build
```

### 6. Source Workspace

Don't forget to source the workspace after doing `catkin build` so that the new
packages wil be recognized.

```sh
source devel/setup.bash  # or devel/setup.zsh
```

## Usage

The main entry point is to use the following command:

```sh
roslaunch ucsd_f1tenth_gazebo main.launch
```
