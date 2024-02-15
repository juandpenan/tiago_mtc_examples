# ROS 2 Tiago MTC Examples Repository

This repository contains examples for the Tiago robot using the ROS 2 Middleware for Control (MTC). Follow the instructions below to set up the environment and run the provided examples.

## Prerequisites

- ROS 2 Humble installed ([ROS 2 Installation](http://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
- Colcon build tool installed ([Colcon Installation](https://colcon.readthedocs.io/en/released/user/installation.html))

## Clone the Repository

Clone this repository and navigate to the root folder:

```bash
git clone https://github.com/your-username/ros2_tiago_mtc_examples.git
cd ros2_tiago_mtc_examples
```

## Pull Third-party Dependencies
Use vcs to pull third-party dependencies specified in thirdparty.repos:

```bash
vcs import src < thirdparty.repos
```
## Install Dependencies with rosdep
Install ROS dependencies using rosdep:

```bash
rosdep install --from-paths src --ignore-src -r -y
```
## Build the Code
Build the code using colcon:

```bash
colcon build --symlink-install
```

## Source the Workspace
Source the workspace to make the built packages available in your environment:

```bash
source install/setup.bash
```
## Launch the Examples
In one terminal, launch the mtc_tutorial_fake:

```bash
ros2 launch tiago_mtc_examples mtc_tutorial_fake.launch.py
```
In another terminal, launch the mtc_executable_fake:

```bash
ros2 launch tiago_mtc_examples mtc_executable_fake.launch.py
```