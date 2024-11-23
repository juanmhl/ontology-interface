#!/bin/bash
docker run -it --rm --user root -v $PWD/ros_package:/catkin_ws/src/inferenceengine:rw --network=host --ipc=host ros-noetic-race