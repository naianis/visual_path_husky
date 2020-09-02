## Overview
This is a developing ROS Package. Ongoing project of a NMPC controller for path following and obstacle avoidance using a Clearpath Husky.
Research founded by FAPESB and hosted on the Federal University of Bahia (UFBA).

This package implements a **proportional line follower considering obstacle avoidance**. The controller performs a semi-circular detour whenever an obstacle is found. For the **NMPC version**, please contact the developer.

## Instalation:
  1. Set the robot model for simulation. Instructions are found here: http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky
  2. Opencv is required
  3. Insert the custom path model in your gazebo enviroment. For that, copy the path_floor folder into your .gazebo/models folder.
  4. Download the package to your workspace
  5. Run catkin_make
  
## Running:
  #### Testing the world:
    > roslaunch visual_path_husky path_husky.launch
  #### Running the world + controller
    > roslaunch visual_path_husky full_husky.launch
