#!/usr/bin/env bash
#
# Copyright 2017 - 2018 Ternaris
# SPDX-License-Identifier: Apache 2.0

for x in /opt/*; do
    if [[ -e "$x/.env.sh" ]]; then
	source "$x/.env.sh"
    fi
done

cd

source /opt/ros/${ROS_DISTRO}/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/${ROS_DISTRO}/share/turtlebot3_gazebo/models

# Enable colcon autocompletion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
