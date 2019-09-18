#!/bin/bash
# setup environment variables for loading meshes in Gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(rospack find baxter_mpnet_with_sensor)/gazebo/env/gazebo_models/
