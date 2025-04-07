# Robot-Assisted Bed Baths for Patients with Limited Mobility - 6-DOF Robot

## Description
This project implements a **obot-Assisted Bed Bath** application using a 6-DOF robot in **Gazebo**. The robot is launched with its controllers, streams camera data, and utilizes a point cloud-based perception pipeline to identify a human. The robot is programmed to pick up a sponge and approach the identified person, simulating an assistive interaction.

## Features
- **Spawn the robot in Gazebo** with all necessary controllers
- **Camera streaming** for computer vision
- **Point cloud processing** to detect and locate a human
- **Motion planning and control** to pick up a sponge and approach the person
- **ROS 2 interface** for system control and monitoring

## Requirements
- **ROS 2 (Humble or later)** make sure to follow the installation guide:https://docs.ros.org/en/humble/Installation.html
- **Gazebo (Classic)**
- **MoveIt 2** for motion planning
- **PCL** for image and point cloud processing
- **RViz** for visualization
- **C++** for ROS nodes

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/jonathansanjay/bed_bath_simulation/
   mkdir your_ws
   cd your_ws
   mkdir src
   colcon build
   cd your_ws/src
   git clone https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
   rosdep update && rosdep install --ignore-src --from-paths . -y
   cd ../
   colcon build
   ```
2. Install the dependencies needed for running the application:
   ```bash
   sudo apt install ros-humble-ros2-control
   sudo apt install ros-humble-ros2-controllers
   sudo apt install ros-humble-gripper-controllers
   sudo apt install gazebo
   sudo apt install ros-humble-gazebo-ros2-control
   sudo apt install ros-humble-gazebo-ros-pkgs
   sudo apt install ros-humble-xacro
   sudo apt install ros-humble-rmw-cyclonedds-cpp
   sudo apt install ros-humble-sensor-msgs
   sudo apt install ros-humble-pcl-conversions
   sudo apt install ros-humble-pcl-ros
   sudo apt install pcl-tools
   ```

3. Copy in your workspace the packages care_robot_sim, cobot_moveit_config and cobot_ik
   ```bash
   cd your_ws
   colcon build
   ```
4. paste these lines in your /.bashrc file in order to load the Gazebo environment variable and the workspace
   ```bash
   source /opt/ros/humble/setup.bash
   source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

   export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
    
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib
   export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
   export GAZEBO_MASTER_URI=http://localhost:11345
   #put your absolute path
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:home/user/your_ws/src/care_robot_sim/models
   source ~/your_ws/install/setup.bash
   ```

## Usage

### 1️⃣ Launch the Simulator
Start Gazebo with the robot and controllers:
```bash
ros2 launch care_robot_sim spawn_moveit.launch.py
```

### 2️⃣ Start the Camera Node and process the PointCloud2 data in order to process and get waypoint with the reference of base_link frama
Begin camera data streaming:
```bash
ros2 run cobot_ik process_points_stream
```

### 3 Run the Pick and Care Sequence
Execute the pick and care routine:
```bash
ros2 run cobot_ik test_pick_care_waypoints
```