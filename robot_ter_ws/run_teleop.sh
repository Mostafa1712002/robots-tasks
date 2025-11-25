#!/bin/bash
# TurtleBot3 Keyboard Control Script

export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash

echo "========================================"
echo "  TurtleBot3 Keyboard Control"
echo "========================================"
echo ""
echo "Controls:"
echo "  i : Move Forward"
echo "  , : Move Backward"
echo "  j : Turn Left"
echo "  l : Turn Right"
echo "  k : Stop"
echo ""
echo "Speed: q/z (all), w/x (linear), e/c (angular)"
echo "CTRL-C to quit"
echo "========================================"
echo ""

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
