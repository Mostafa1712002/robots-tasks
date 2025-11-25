# Task 1 – Custom Gazebo World with TurtleBot3 Teleop

This repository contains a ROS 2 (Jazzy) workspace that launches a custom Gazebo world, spawns a TurtleBot3, and lets you drive it with `teleop_twist_keyboard`.

## 1. Workspace layout

```
robot_ter_ws/
  src/
    robot_ter_bringup/
      launch/custom_world.launch.py   # Gazebo + TurtleBot3 launcher
      worlds/robot_ter_city.world     # Custom arena with obstacles
```

## 2. Prerequisites

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-gazebo-ros-pkgs \
     ros-jazzy-teleop-twist-keyboard ros-jazzy-turtlebot3-description \
     ros-jazzy-xacro
```

Set the TurtleBot3 model once per shell (Burger matches the launch defaults):

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 3. Build the workspace

```bash
cd robot_ter_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## 4. Launch the custom world and spawn the bot

```bash
ros2 launch robot_ter_bringup custom_world.launch.py
```

What you get:
- Gazebo opens with the `robot_ter_city` world (spawn pad, warehouse block, pillars, charging station, and pedestrian zone to plan around).
- A TurtleBot3 Burger is spawned at the origin with simulated time enabled.

To switch models (e.g., Waffle Pi):

```bash
ros2 launch robot_ter_bringup custom_world.launch.py \
  turtlebot3_model_file:=turtlebot3_waffle_pi.urdf.xacro
```

## 5. Drive the robot with teleop keys

In another terminal (after sourcing `install/setup.bash`), run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Use the on-screen WASD bindings to move TurtleBot3 throughout the custom world and complete Task 1.
# robots-tasks
