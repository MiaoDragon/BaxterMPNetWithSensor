cmake_minimum_required(VERSION 2.8.3)
project(baxter_mpnet_with_sensor)

catkin_python_setup()
install(PROGRAMS
  point_cloud_merger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
