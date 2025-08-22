Differential Drive Robot Simulation on ROS 2 Humble 🤖
This is a simulation project for a differential drive robot using ROS 2 Humble and Gazebo.
► Prerequisites
Before you begin, ensure you have the following installed:
Ubuntu 22.04
ROS 2 Humble Hawksbill
Gazebo (typically installed with ros-humble-desktop)
Colcon (the standard build tool for ROS 2)
Git
► Installation and Build Instructions
Follow these steps to set up and build the project.
1. Create a Workspace and Clone the Project
Open a terminal and run the following commands:
code
Bash
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
# Navigate to the root of your workspace
cd ~/ros2_ws

# Build the project
colcon build
A successful build will end with a message like: Summary: 1 package finished.
► Usage Instructions
After a successful build, you can launch the simulation.
1. Source the Workspace
In every new terminal session, you must source the workspace environment so that ROS 2 can find your packages.
code
Bash
# Source the workspace's setup file
source ~/ros2_ws/install/setup.bash
Note: You must run this command in every new terminal where you intend to use this workspace.
2. Launch the Simulation
Now, use the ros2 launch command to start the simulation in Gazebo.
code
Bash
ros2 launch diffbot_sim diffbot.launch.py
A Gazebo window will open with the robot model. You can now start interacting with and controlling the robot through ROS 2 topics.
