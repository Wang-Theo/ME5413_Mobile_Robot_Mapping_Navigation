#Report: Group18_FinalProject.pdf

#PPT: Presentation.pptx

#Video Recording: 
The complete and high quality uncompressed file can be found in:
----------------------------------------------------------------------------------------------------	
https://drive.google.com/file/d/1lj-83mchuxrjdUUB504Vcz-bM367FJpr/view?usp=share_link
-------------------------------------------------------------------------------------------------------

#Map file
A 2D projection map file is: map.pgm

one version of the 3D pointcloud mapping files can be download in
------------------------------------------------------------------------------------------------------------
https://drive.google.com/file/d/19SF8SPli2iWed0avGKpOY63t1eXppWkJ/view?usp=share_link
------------------------------------------------------------------------------------------------------------

# Operation guide

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 16.04**

For **Ubuntu 18.04 or higher**, the **default** PCL and Eigen is enough for FAST-LIO to work normally.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### 1.3. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

*Remarks:*
- Since the FAST-LIO must support Livox serials LiDAR firstly, so the **livox_ros_driver** must be installed and **sourced** before run any FAST-LIO luanch file.
- How to source? The easiest way is add the line ``` source $Licox_ros_driver_dir$/devel/setup.bash ``` to the end of file ``` ~/.bashrc ```, where ``` $Licox_ros_driver_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).

### 1.4 robot-pose-ekf
```
sudo apt-get install ros-noetic-robot-pose-ekf
```

### 1.5 navigation global_planner
```
sudo apt-get install ros-noetic-navigation
```

### 1.6 folder needed
create a folder named `include` in`src/aster_ws/src/Astar_planner`. And an empty folder named`astar_planner` in it.

## 2. Execution
### Part 1 Mapping
![mapping_result_image](https://github.com/Wang-Theo/ME5413_Final_Project/blob/main/report/images/mapping.png)

1) Initialize FAST-LIO mapping

```
cd ~/ME5413_Final_Project
catkin_make

cd ~/ME5413_Final_Project/src/FAST_LIO_
mkdir PCD

# First terminal
source devel/setup.bash
roslaunch me5413_world world.launch

# Second terminal
source devel/setup.bash
roslaunch me5413_world fast_lio.launch

# Third  terminal (rosbag for EVO)
cd ~/ME5413_Final_Project/EVO
rosbag record /gazebo/ground_truth/state /Odometry -o EVO_perform.bag
```
After doing mapping, pointcloud `scans.pcd` will save in `src/FAST_LIO_/PCD/`   
Using EVO to evaluate the mapping performence : `evo_ape bag EVO_perform.bag /gazebo/ground_truth/state /Odometry -r full -va --plot --plot_mode xy`

2) Convert pcd pointcloud to grid map

Firstly, change the filepath in `pcdtomap.launch` to your own path (in the `src/pcdtomap/launch/`)
```
# One terminal
source devel/setup.bash
roslaunch pcdtomap pcdtomap.launch

# Another terminal
cd ~/ME5413_Final_Project/src/pcdtomap/map/
rosrun map_server map_saver
```
The pointcloud file after filtering `map_radius_filter.pcd` is saved in `src/FAST_LIO_/PCD/`   
The grid map file `map.pgm` and `map.yaml` is saved in `src/pcdtomap/map/`   
We backup copy the good result in the `/backup` folder

### Part 2 Navigation
![naviagation_result_image](https://github.com/Wang-Theo/ME5413_Final_Project/blob/main/report/images/navigation_result.png)

1) Initialize navigation
```
cd ~/ME5413_Final_Project
catkin_make
```
Uncomment corresponding algorithm you want to use in `src/me5413_world/launch/move_base.launch`
The parameters of planning algorithms and costmap are in corresponding `params` files.
For localization, amcl, ekf_template and robot_pose_ekf had been used. Then, the global planner has three choice can be select, finally, the local planner has two method had been provide. Pick the corrspending choice and comment specific command in those files can implement them.
To start, 
```
# First terminal
source devel/setup.bash
roslaunch me5413_world world.launch

# Second terminal
source devel/setup.bash
roslaunch me5413_world navigation.launch
```
Choose corresponding topic in `global path` and click the button in `simplePanel` to select the goal pose.
