#!/bin/bash
# A wrapper script to enforce use_sim_time:=true for all ROS2 nodes launched by workflow_node.
# This prevents ROS args parsing errors caused by python appending arguments, by ensuring
# the use_sim_time argument is appended at the very end of the command line.

exec "$@" --ros-args -p use_sim_time:=true
