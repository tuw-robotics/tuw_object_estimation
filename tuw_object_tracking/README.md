tuw_object_tracking
===

Particle filter based algorithm to track objects in 3D coordinates from a mobile robot. 
Currently, this package is solely used to track people using detection algorithms from [tuw_people_detection](https://github.com/tuw-robotics/tuw_people_detection).
Nevertheless, it can in principle be used to track any object given appropriate detections and motion models.

## Heat Map Motion Prediction

Besides constant velocity and coordinated turn forward prediction, the `tuw_object_tracking` package implements a heat map motion model based on heat maps created by the `tuw_tracking_heatmap` package. Based on such heat maps, the underlying static map as well as historical data can be considered. The image below shows an example visualization of a forward prediction. The blue arrows show particles in the current state, the red ones are predicted 0.1 s into the future and the green ones 0.2s into the future. One can observe that predictions cover different possible direction, whereas straight forward remains the most likely. 

<img src="https://github.com/tuw-robotics/tuw_object_estimation/blob/master/tuw_object_tracking/res/particle_prediction.png" width="500"/>

## Installation:

### ROS source package dependencies:
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

No special compilation flags required. However, for performance reasons one should compile with ```-DCMAKE_BUILD_TYPE=Release```.
    
## Compatible Detection Algorithms:

Generally, arbitrary detection algorithms can be used as long as they provide detections through
```ObjectDetection``` messages provided in [tuw_msgs](https://github.com/tuw-robotics/tuw_msgs).
Ready to use detectors for people detection are implemented in [tuw_people_detection]().

## Usage:

The ```config``` directory contains separate yaml configuration files for detectors used, as well as ROS parameters for the tracking algorithm.
As an example ```detection_topics_rgbd_laser_rgb.yaml``` contains an array holding topic names of available detectors. 
An example for tracking parameters is provided in ```parameters_heatmap.yaml``` using the heat map motion model and ```parameters_cv.yaml``` using a constant velocity forward prediction.

Launch with:
```
roslaunch tuw_object_tracking tuw_people_tracking.launch
```

## Demo:

A bag file demo is available using pre-recorded leg detections / YOLOv3 or YOLOv3-tiny detections.
First the bag files have to be downloaded and decompressed:
```
roscd tuw_object_tracking/bags/people-tracking-roblab
wget -r -nH --cut-dirs=2 --no-parent -e robots=off  --reject="*index.html*" http://roblab.auto.tuwien.ac.at/ros-bagfiles/people-tracking-roblab/
rosbag decompress 2018-09-25_roblab_detection*
```
Then the demo (including an rviz configuration) can be launched with:
```
roslaunch tuw_object_tracking people_tracking_bag_demo.launch
```
To use YOLOv3-tiny detections instead use:
```
roslaunch tuw_object_tracking people_tracking_bag_demo.launch tiny_yolo:=true
```
