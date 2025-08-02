# ROS 2 Differential-Drive Robot Simulation

This project contains a ROS 2 (Humble) simulation of a differential-drive mobile robot in Gazebo. The robot is controllable via keyboard and its physical and performance parameters can be configured through a YAML file.

---

## Requirements

*   Ubuntu 22.04
*   ROS 2 Humble Hawksbill
*   Gazebo (Modern Gazebo / Ignition)
*   `ros-humble-ros-gz` for ROS-Gazebo integration
*   `ros-humble-teleop-twist-keyboard` for manual control
*   `python3-pandas`, `python3-matplotlib` for performance analysis

---

## How to Build

1.  Clone this repository into your `ros2_ws/src` directory.
2.  Navigate to the root of your workspace (`~/ros2_ws`).
3.  Install dependencies (if needed):
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
4.  Build the package:
    ```bash
    colcon build --packages-select diffbot_sim
    ```

---

## How to Run

1.  Source your workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2.  Launch the main simulation file. This will open Gazebo, RViz, and a teleoperation terminal:
    ```bash
    ros2 launch diffbot_sim simulation.launch.py
    ```

3.  Click on the `xterm` window that opens and use the keyboard keys (`i`, `j`, `k`, `l`, etc.) to drive the robot.

---

## How to Test Performance

The project includes a script to verify the robot's performance (0-5m/s in 5s, stop in 1s).

1.  Run the automated test script. This will start recording data, run the robot through an acceleration/deceleration cycle, and save the data to a `.bag` file in `~/ros2_ws/performance_bags/`.
    ```bash
    # Make sure the simulation is running first
    ./performance_test.sh 
    ```

2.  Analyze the recorded data using the provided analysis script. This will generate a plot and print the results to the terminal.
    ```bash
    # Make sure to update the bag file path inside the script
    python3 ~/ros2_ws/ana.py
    ```

---

## How to Extend (Modify Parameters)

The robot's physical and performance parameters can be easily modified without changing the source code.

1.  Open the configuration file: `config/diffbot_params.yaml`.
2.  Change any parameter, for example:
    ```yaml
    # config/diffbot_params.yaml
    ...
    max_linear_acceleration: 2.0 # Make the robot accelerate faster
    wheel_radius: 0.5            # Make the wheels bigger
    ...
    ```
3.  Save the file, rebuild the package (`colcon build`), and relaunch the simulation to see the changes.
