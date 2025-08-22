Chắc chắn rồi! Dưới đây là nội dung file README đã được định dạng bằng Markdown để trông chuyên nghiệp và dễ đọc hơn trên GitHub.

Bạn chỉ cần sao chép toàn bộ khối mã bên dưới và dán vào file `README.md` của bạn.

---

# Differential Drive Robot Simulation on ROS 2 Humble 🤖

This is a simulation project for a differential drive robot using **ROS 2 Humble** and **Gazebo**.

## ► Prerequisites

Before you begin, ensure you have the following installed:
*   Ubuntu 22.04
*   ROS 2 Humble Hawksbill
*   Gazebo (typically installed with `ros-humble-desktop`)
*   Colcon (the standard build tool for ROS 2)
*   Git

---

## ► Installation and Build Instructions

Follow these steps to set up and build the project.

### 1. Create a Workspace and Clone the Project

Open a terminal and run the following commands:

```bash
# Create a new workspace directory and its src folder
mkdir -p ~/ros2_ws/src

# Navigate into the src directory
cd ~/ros2_ws/src

# Clone the repository from GitHub
git clone https://github.com/Tranthanhbao198/ros2_diffbot_simulation.git
```

### 2. Install Dependencies

Before building, you need to install the necessary packages for simulation and control.

```bash
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
```

### 3. Build the Workspace

Use `colcon` to build all packages in the workspace.

```bash
# Navigate to the root of your workspace
cd ~/ros2_ws

# Build the project
colcon build
```
A successful build will end with a message like: `Summary: 1 package finished`.

---

## ► Usage Instructions

After a successful build, you can launch the simulation.

### 1. Source the Workspace

In every new terminal session, you must source the workspace environment so that ROS 2 can find your packages.

```bash
# Source the workspace's setup file
source ~/ros2_ws/install/setup.bash
```
> **Note:** You must run this command in **every new terminal** where you intend to use this workspace.

### 2. Launch the Simulation

Now, use the `ros2 launch` command to start the simulation in Gazebo.

```bash
ros2 launch diffbot_sim diffbot.launch.py
```

A Gazebo window will open with the robot model. You can now start interacting with and controlling the robot through ROS 2 topics.
Of course! Here is that section translated into English, formatted with Markdown to continue directly from the previous part of your `README.md` file.

---

### 3. Controlling the Robot with a Keyboard (Teleop)

Once the simulation is running, you can control the robot's movement using the `teleop_twist_keyboard` package. There are two ways to do this.

#### Method 1: Run Teleop in a Separate Terminal (Recommended for Testing)

This method is quick and easy, perfect for verifying that your robot is working correctly.

**A. Launch the Simulation**

First, make sure your simulation is running. Open **Terminal 1**, source your workspace, and run the launch file:```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch diffbot_sim diffbot.launch.py
```

**B. Launch Keyboard Teleop**

Open a **NEW Terminal (Terminal 2)**, source the workspace, and run the `teleop_twist_keyboard` node. We need to **remap** the default `cmd_vel` topic to the topic our robot's controller is listening to, which is `/diff_cont/cmd_vel_unstamped`.

```bash
# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_cont/cmd_vel_unstamped
```

**C. Start Driving**

1.  **Click on the Terminal 2 window** to give it **focus**.
2.  Use the following keys to control the robot in Gazebo:
    ```
           i
        j     l
           ,
    ```
    - `i`: move forward
    - `,`: move backward
    - `j`: turn left
    - `l`: turn right
    - `u/o/m/.`: move diagonally
    - `k`: stop
    - `q/z`: increase/decrease speed

---
#### Method 2: Integrate into a Launch File (Advanced)

This method allows you to start both the simulation and the teleop node with a single command.

**A. Create a New Launch File**

Create a new file named `full_sim.launch.py` inside your package's `launch` directory.
```bash
# From your workspace root (~/ros2_ws)
cd src/ros2_diffbot_simulation/diffbot_sim/launch/
touch full_sim.launch.py
```

**B. Add Content to the Launch File**

Open `full_sim.launch.py` and paste the following content into it:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot simulation launch file
    diffbot_launch_path = os.path.join(
        get_package_share_directory('diffbot_sim'),
        'launch',
        'diffbot.launch.py'
    )

    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diffbot_launch_path)
    )

    # Launch the teleop_twist_keyboard node in a new terminal window
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_node',
        output='screen',
        prefix='xterm -e',  # This opens a new xterm terminal for the node
        remappings=[
            ('cmd_vel', '/diff_cont/cmd_vel_unstamped')
        ]
    )

    return LaunchDescription([
        included_launch,
        teleop_node
    ])
```
> **Explanation:** The `prefix='xterm -e'` argument is very important. It automatically opens a new terminal window just for the teleop node, which is necessary because the node needs to capture your keyboard input.

**C. Rebuild the Workspace**

Since you added a new file, rebuild your package so the system can find it.
```bash
# Return to the workspace root
cd ~/ros2_ws
colcon build --packages-select diffbot_sim
```

**D. Run the Combined Launch File**

Now, you only need to run this new launch file.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch diffbot_sim full_sim.launch.py
```
A Gazebo window and a new terminal window will open automatically. **Click on the new terminal window** and start driving the robot.

---

### ⚙️ Troubleshooting: Robot Doesn't Move

If you press the keys but the robot doesn't move, check these common issues:
1.  **Terminal Not in Focus**: This is the most common problem. Make sure you have clicked on the terminal window running the teleop node before pressing keys.
2.  **Commands Are Not Being Sent**: Open a third terminal and run `ros2 topic echo /diff_cont/cmd_vel_unstamped`. Then go back to the teleop terminal and press keys. If you don't see any messages in the third terminal, the command is not being sent (see issue #1).
3.  **Controller is Inactive**: Open another terminal and run `ros2 control list_controllers`. Ensure that `diff_cont` has an `active` state.
