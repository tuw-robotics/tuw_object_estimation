tuw_object_tracking
===

Particle filter based algorithm to track objects in 3D coordinates from a mobile robot. 
Currently, this package is solely used to track people using detection algorithms from [tuw_people_detection]().
Nevertheless, it can in principle be used to track any object given appropriate detections and motion models.


# Installation:

## Dependencies:

### ROS source packages:
  roscpp  
  tuw_object_msgs  
  tuw_geometry  
  geometry_msgs  
  tf  
  dynamic_reconfigure  
  grid_map_msgs  
  grid_map_ros  
  grid_map_cv  
  grid_map_core  
  pose_cov_ops  

* [tuw_common](https://github.com/tuw-robotics/tuw_common) ```check the INSTALL.md inside the pkg because it holds submodules```
  * [tuw_msgs](https://github.com/tuw-robotics/tuw_msgs)
  * [tuw_geometry](https://github.com/tuw-robotics/tuw_geometry)

* [grid_map](https://github.com/tuw-robotics/grid_map)```git clone git@github.com:tuw-robotics/grid_map.git```
  * for a minimal installation you can exclude some pkgs 
    * touch ./src/grid_map/grid_map_rviz_plugin/CATKIN_IGNORE
    * touch ./src/grid_map/grid_map_demos/CATKIN_IGNORE
    * touch ./src/grid_map/grid_map_visualization/CATKIN_IGNORE
    * touch ./src/grid_map/grid_map_loader/CATKIN_IGNORE

### Compilation:
No special compilation flags required. However, for performance reasons one should compile with ```-DCMAKE_BUILD_TYPE=Release```.
    
# Compatible Detection Algorithms:

Generally, arbitrary detection algorithms can be used as long as they provide detections through
```ObjectDetection``` messages provided in [tuw_msgs](https://github.com/tuw-robotics/tuw_msgs).
Ready to use detectors for people detection are implemented in [tuw_people_detection]().

# Usage:

## Configuration:

The ```config``` directory contains separate yaml configuration files for detectors used, as well as ROS parameters for the tracking algorithm.
As an example ```detection_topics_rgbd_laser_rgb.yaml``` contains an array holding topic names of available detectors. 
An example for tracking parameters is provided in ```parameters_heatmap.yaml``` using the heat map motion model.

## Launch:

Launch with:
```
roslaunch tuw_object_tracking tuw_people_tracking.launch
```
