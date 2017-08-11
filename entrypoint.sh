#!/bin/bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source devel/setup.bash
roscd ig_action_server/src
python ig_action_server.py
