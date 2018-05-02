tuw_tracking_heatmap
===

This package creates heat maps of likely locations of people in a static map. It is able to compute an initial likelihood field based on a static map and furthermore, observed tracks can be integrated for the use of historic data.

# Packages needed in workspace
* grid_map https://github.com/tuw-robotics/grid_map
* tuw_common https://github.com/tuw-robotics/tuw_common

# Compilation
Compile with 
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```
Otherwise everything will be really slow

# Usage

Run with
```
roslaunch tuw_tracking_heatmap tracking_heatmap.launch
```

Initially, when there is no previously recorded bagfile available, the node uses the received OccupancyGrid map to compute an initial likelihood field, of where people are likely to be encountered. Next, it listens to the `tracked_persons` topic and increases the likelihood of people at the received locations. All of the data is stored in different layers of a `grid_map` which is periodically saved into a bagfile. 

When the node is provided a previously recorded bagfile, it merges the initial likelihood field layer with the likelihood obtained from tracked persons. Furthermore, it computes a vector field, which is used to predict a persons path in the MPN framework. In addition the node can optionally compute the static portion of social forces as proposed in [1]. 


# References:
[1] M. Luber, J. A. Stork, G. D. Tipaldi, and K. O. Arras. People tracking with
human motion predictions from social forces. In 2010 IEEE International
Conference on Robotics and Automation, pages 464–469, May 2010
