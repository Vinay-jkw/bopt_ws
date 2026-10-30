#!/bin/bash
# A wrapper script to enforce use_sim_time:=true for all ROS2 nodes launched by workflow_node.
# If --ros-args is already in the argument list, only append -p use_sim_time:=true.
# Otherwise, append --ros-args -p use_sim_time:=true.

has_ros_args=false
for arg in "$@"; do
    if [ "$arg" = "--ros-args" ]; then
        has_ros_args=true
        break
    fi
done

if [ "$has_ros_args" = true ]; then
    exec "$@" -p use_sim_time:=true
else
    exec "$@" --ros-args -p use_sim_time:=true
fi
