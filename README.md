Of course! Here is the README.md file translated into English.

Differential Drive Robot Simulation on ROS 2 Humble

This is a simulation project for a differential drive robot using ROS 2 Humble and Gazebo.

Prerequisites

Ubuntu 22.04

ROS 2 Humble Hawksbill

Gazebo (installed with ros-humble-desktop)

Colcon (the standard build tool for ROS 2)

Git

Installation and Build Instructions

Follow these steps to set up and build the project.

1. Create a Workspace and Clone the Project

Open a terminal and run the following commands:

code
Bash
download
content_copy
expand_less

# Create a new workspace directory and its src folder
mkdir -p ~/ros2_ws/src

# Navigate into the src directory
cd ~/ros2_ws/src

# Clone the repository from GitHub
git clone https://github.com/Tranthanhbao198/ros2_diffbot_simulation.git
2. Install Dependencies

Before building, you need to install the necessary packages for simulation and control.

code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
# Navigate to the root of your workspace
cd ~/ros2_ws

# Update your system's package list
sudo apt update

# Install the required dependencies
sudo apt-get install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control
3. Build the Workspace

Use colcon to build all packages in the workspace.

code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
# Navigate to the root of your workspace
cd ~/ros2_ws

# Build the project
colcon build

If the build process is successful, you will see a summary line like: Summary: 1 package finished.

Usage Instructions

After a successful build, you can launch the simulation.

1. Source the Workspace

In every new terminal, you must source the workspace environment so that ROS 2 can find your newly built packages.

code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
# Source the workspace's setup file
source ~/ros2_ws/install/setup.bash

Note: You must run this command in every new terminal where you intend to use this workspace's commands.

2. Launch the Simulation

Now, use the ros2 launch command to start the simulation in Gazebo.

code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
ros2 launch diffbot_sim diffbot.launch.py

A Gazebo window will open with the robot model. You can now start interacting with and controlling the robot through ROS 2 topics.
