/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by                                                 *
 *   Klaus Buchegger <klaus.buchegger@student.tuwien.ac.at>                *
 *   Florian Beck    <florian.beck@tuwien.ac.at>                           *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tuw_object_msgs/ObjectDetection.h>
#include <tuw_object_msgs/ObjectWithCovarianceArray.h>
#include <tuw_object_msgs/ObjectWithCovarianceArrayArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>

namespace tuw
{
class TrackingHeatmapNode
{
public:
  /*!
   * Constructor
   * 
   * @param n ROS node handle
   */
  TrackingHeatmapNode(ros::NodeHandle& n);
  
  /*!
   * Publish heat map as grid_map message
   */
  void publish(void);
  ros::NodeHandle n_;        ///< Node handler to the root node
  ros::NodeHandle n_param_;  ///< Node handler to the current node

private:
  /*!
   * Callback for received tracks as ObjectDetection messages.
   * Constructs a histogram of people's location in the map from the tracks
   * applying linear interpolation between time stamps. 
   * 
   * @param _tracked Received tracks of people
   */
  void trackingCallbck(const tuw_object_msgs::ObjectDetection::ConstPtr& _tracked);
  
  /*!
   * Callback for the static map.
   * Initiallizes a grid_map based on the received static map's size and resolution.
   * This also calls be function to compute the initial likelihood field.
   * Writes the generated grid_map into the bag file for backup.
   * 
   * @param _globalMap Received occupancy grid map.
   */
  void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& _globalMap);
  
  /*!
   * Periodically calls publish in order to publish the grid map.
   * 
   * @param event unused
   */
  void timerCallback(const ros::TimerEvent& event);
  
  /*!
   * Periodically saves the recorded data to the bag file.
   * 
   * @param event unused
   */
  void savingTimerCallback(const ros::TimerEvent& event);
  
  /*!
   * clears the internal grid map
   */
  void clearMap(void);
  
  /*!
   * Precomputes a vector field of directions representing the likelihood of directions.
   * This is not used in people tracking, but in human motion prediction for navigation.
   */
  void precomputeVectorField(void);
  
  /*!
   * Computes the initial likelihood field based on distances to walls in the static map.
   */
  void computeInitialLikelihoodField(void);
  
  /*!
   * Computes static part of the social force model, i.e. that walls apply repelling forces.
   */
  void computeSocialForces(void);

  ros::Subscriber subTrackings_;
  ros::Subscriber subGlobalMap_;
  ros::Publisher pubGridMap_;
  ros::Publisher pubProbVectorField_;
  ros::Publisher pubHistTrajectories_;
  std::vector<ros::Publisher> pubVectorField_;

  ros::Timer timer_;
  ros::Timer savingTimer_;

  grid_map::GridMap map_;
  std::vector<grid_map::Matrix*> layers_;

  std::map<int, geometry_msgs::Pose> lastPoses_;
  tf::TransformListener tf_listener_;
  visualization_msgs::MarkerArray vectorField[4];

  std::string grid_map_bagfile_;
  std::string grid_map_bagfile_out_;
  std::string image_output_dir_;
  
  std::map<int, tuw_object_msgs::ObjectWithCovarianceArray> hist_trajectories_;
  tuw_object_msgs::ObjectWithCovarianceArrayArray hist_trajectories_array_of_arrays_;

  bool apply_blur_;
  int ksize_;
  double likelihood_field_sigma_;

  // social forces stuff
  bool compute_social_forces_;
  double a_obj_;
  double b_obj_;
  double c_obj_;
  double r_pers_;
  int border_length_;
  double scale_;
  double person_weight_;
};
}
