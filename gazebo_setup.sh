# setup environment variables for loading meshes in Gazebo
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:$(dirname $(realpath -s ${BASH_SOURCE[0]}))/gazebo/env/gazebo_models/"
