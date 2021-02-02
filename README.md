# Robot-Path-Planing

For an Occupancy Grid Map output from a SLAM algorithm, implement either __A* path planning__ algorithm for a specified goal position as a service provider (provide the goal
position as input and obtain the path to goal as response). 

Use the LiDAR data for generating the map with the SLAM node in ROS.

## How to run

1. Unzip folder hao_a3 to 
>    ~/catkin_ws/src/ 

2. Build the pkg: 
>   ~/catkin_ws $ catkin_make
>   ~/catkin_ws $  source devel/setup.bash

3. Open 3 terminal and run below scripts in each. 
     astar.py is the service server. 
     value_it.py is the service client. 
>   ~$ roscore
>   ~$ rosrun hao_a3 astar.py 
>   ~$ roslaunch hao_a3  a3.launch 

4. Choose 2D Nav Goal in RVIZ, click one point on the map. 
    Then the path will show up. 
    It will take longer if the path is very long. 
    

## Result

__Green dot__:  Current position.

**Black dot**:  Destination.

<div align="center">
  <img src="https://github.com/glennjw/Robot-Path-Planing/blob/main/Result.png">
</div>

