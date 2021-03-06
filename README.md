# Map-My-World-RSE
Create a 2D occupancy grid and 3D octomap from a simulated environment using your own robot with the RTAB-Map package.

## Task
* ROS Package: robot and RTABMAP
* Student's simulation world and robot could properly load in Gazebo.
* The student's environment should have clear features and geometric shapes to perform mapping.
* Student's map should contain at least 3 loop closures and the occupancy grid is identifiable.
* Db file generated (could be link to file if oversized)

## Folder Structure
* **Map-My-World-RSE**
    * **catkin_ws**
        * **src**
            * **ira_laser_tools** - Submodule, package to merge two laser scans (My robot has two 2-D lidar).
            * **teleop_twist_keyboard** - Tele-operation package.
            * **my_robot**
                * **config** - Contains config files for rtabmap_viz and also navigation config from [Where-Am-I-RSE](https://github.com/prasun2712/Where-Am-I-RSE).
                * **launch** - Contains all the launch files.
                    * **merger_scan.launch** - Launch file to merge two laser scan data.
                    * **skid_steer_diff_bot_base_laser.launch** - Launch file to load robot xacro and run robot_state_publisher and joint_state_publisher.
                    * **teleop.launch** - Launch file for teleop node.
                    * **world.launch** - Launch file to spawn robot in world.
                    * **robot_localization.launch** - Launch file to bringup simulated world with robot and rtabmap node in localization mode with rviz.
                    * **robot_mapping.launch** - Launch file to bringup simulated world with robot and rtabmap node along with rtabmap visualiser.
                    * **rtab_map_localize.launch** - Launch file for rtabmap node in localization mode.
                    * **rtab_map.launch** - Launch file for rtabmap node and rtabmap viz.
                    * amcl.launch - Launch file for amcl.
                    * map_server.launch - Launch file for map server.
                    * move_base.launch - Launch file for move_base node and load params.
                    * robot_navigation.launch - Launch file to bringup navigation.
                * **maps** - Contains map file and map metadata.
                * **meshes** - Contains mesh files.
                * **urdf** - Contains robot related xacros.
                * **world** - Contains my world file.
    * **images** - Assignment images.

## Images for assignment
|World View   |rviz_cloudmap+gridmap  |rviz_cloudmap |rtabmap_viz    |
| -------------- |  :---------   |  ----------:       |    :----------:              |
| ![](https://github.com/prasun2712/Map-My-World-RSE/blob/main/images/gazebo_world_with_robot.png) | ![](https://github.com/prasun2712/Map-My-World-RSE/blob/main/images/rviz_cloudmap_with_gridmap.png) | ![](https://github.com/prasun2712/Map-My-World-RSE/blob/main/images/rviz_cloudmap.png) | ![](https://github.com/prasun2712/Map-My-World-RSE/blob/main/images/rtab_map_viz.png) |

#### Global Loop Closure in Gridmap View.
As shown in the image below, total number of **global loop closure** &#8594; **9**. Minimum required was 3.
![](https://github.com/prasun2712/Map-My-World-RSE/blob/main/images/global_loop_closure.png) 

## RTABMAP Database Files Link
* **my_world.world &#8594;** [rtabmap.db](https://drive.google.com/file/d/1tTmys7Zl_nkQRHaSpnQGJRgb1b3ZPhC-/view?usp=sharing)
* Visualise database using :
```
rtabmap-databaseViewer <downloaded_database_file>
```
* Once open :
    * Say yes to using the database parameters
    * View &#8594; Graph view
    * View &#8594; Occupancy Grid

## Prerequisite
* Basic knowledge of ROS.
* ROS Kinetic installed.
* ros-kinetic-rtabmap, ros-kinetic-rtabmap-ros, ros-kinetic-navigation, ros-kinetic-map-server, ros-kinetic-move-base, ros-kinetic-amcl, ros-kinetic-dwa-local-planner

## Build and Run
```
cd ~/
git clone https://github.com/prasun2712/Map-My-World-RSE.git
cd ~/Map-My-World-RSE/
git submodule init
git submodule update
cd ~/Map-My-World-RSE/catkin_ws
catkin_make
```

### Mapping
***Terminal 1 - Run rtabmap to map the environment.***
```
roslaunch my_robot robot_mapping.launch
```
***Terminal 2 - Teleop.***
```
roslaunch my_robot teleop.launch
```

### Localisation
Before this download the rtabmap database file from the link provided above and put it in the maps folder.

***Terminal 1 - Run rtabmap for localisation***
```
roslaunch my_robot robot_localization.launch
```
The robot will start from the last position it was on when the db was saved and will localise itself as we teleop.
***Terminal 2 - Teleop.***
```
roslaunch my_robot teleop.launch
```