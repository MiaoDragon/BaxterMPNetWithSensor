This project uses Baxter for both simulated environment (Gazebo) and real-world

The pipeline of this project is the following:
1. generate point cloud in simulated environment or real-world, and save it to data/pcd
  This part concatenates point clouds captured by multiple cameras, and remove the robot point cloud
  by firstly filtering out nearby points, and then using robot_self_filter package

  To run this part in simulator, run the following:
    ```
    roslaunch baxter_gazebo.launch
    (make sure this step is successful until the info: Gravity compensation was turned off)
    
    roslaunch gazebo_point_cloud_generation.launch
      -- this spawns multiple cameras in Gazebo environment, and set up point cloud merger and self_filter      
      
    python pc_generation/gazebo_point_cloud_saver.py
      -- this uses the executable 'pointcloud_to_pcd' from PCL ROS package to store point cloud into pcd file
         it loads the environment file in gazebo/env/environment_data   
    ```
    
2. generate paths using the saved point cloud
  This part loads the previously saved point clouds, and puts it in the MoveIt planning scene by using
  MoveIt built-in ROS package. See tutorial:
    http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/perception_pipeline/perception_pipeline_tutorial.html
  for more information.

  To run this part in simulator, run the following:
    roslaunch baxter_gazebo.launch
    roslaunch baxter_plan_with_sensor.launch
      -- this sets the necessary arguments for 3D perception (for instance, point cloud ROS topic), and sets up
         MoveIt packages for planning (joint_trajectory_action_server and baxter_grippers.launch)
    python load_pointcloud_plan.py
      -- this loads the pointcloud data, and uses MoveIt python binding for obtaining the path data,
         the path data is then saved in data/path
