/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Florian Beck <florian.beck@tuwien.ac.at>        *
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

#ifndef TUW_OBJECT_TRACKING_NODE_H
#define TUW_OBJECT_TRACKING_NODE_H

#include <ros/ros.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include "object_tracker.h"
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_object_tracking/tuw_object_trackingConfig.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>

class ObjectTrackingNode : public ObjectTracker
{
public:
  ObjectTrackingNode(ros::NodeHandle& nh, std::shared_ptr<ParticleFilterConfig> pf_config, std::shared_ptr<TrackerConfig> t_config);
  void callbackParameters(tuw_object_tracking::tuw_object_trackingConfig &config, uint32_t level);
  void detectionCallback(const tuw_object_msgs::ObjectDetection& detection);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void printTracks(bool particles);
  void publishTracks(bool particles);
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_detection_;
  ros::Subscriber sub_initial_pose_;
  ros::Subscriber sub_occupancy_map_;
  std::vector<std::string> detection_topics_;
  std::vector<ros::Subscriber> sub_vector_;
  ros::Publisher pub_detection_;
  ros::Publisher pub_detection_immature_;
  ros::Publisher pub_particles_;
  ros::Publisher pub_gridmap_;
  ros::Publisher pub_cluster_centroids_;
  
  dynamic_reconfigure::Server<tuw_object_tracking::tuw_object_trackingConfig> reconfigureServer_;
  dynamic_reconfigure::Server<tuw_object_tracking::tuw_object_trackingConfig>::CallbackType reconfigureFnc_;
  
  tuw_object_tracking::tuw_object_trackingConfig config_;
  
  std::shared_ptr<tf::TransformListener> tf_listener_;
  
  std::string common_frame_;
  std::string grid_map_bagfile_;
  
  grid_map::GridMap heat_map_;
  grid_map::Matrix* layer_;
  float grid_map_max_;
  float grid_map_min_;
};

#endif  // TUW_OBJECT_TRACKING_NODE_H
