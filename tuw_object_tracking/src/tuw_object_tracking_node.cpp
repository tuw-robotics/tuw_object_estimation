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

#include "tuw_object_tracking_node.h"
#include <const_vel_system_model.h>
#include <heatmap_system_model.h>
#include <heatmap_system_model_inv.h>
#include <const_acc_system_model.h>
#include <coordinated_turn_system_model.h>
#include <simple_meas_model.h>
#include <mahalanobis_meas_model.h>
#include <mahalanobis_meas_model_inv.h>
#include <tuw_geometry/tuw_geometry.h>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <rviz/ogre_helpers/arrow.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <pose_cov_ops/pose_cov_ops.h>
#include "object_tracker_separate.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/highgui.hpp>


#define foreach BOOST_FOREACH

using std::cout;
using std::endl;

ObjectTrackingNode::ObjectTrackingNode(ros::NodeHandle& nh, std::shared_ptr<ParticleFilterConfig> pf_config,
                                       std::shared_ptr<TrackerConfig> t_config)
  :  nh_(nh), nh_private_("~")
{
  // generate subscribers to topics in detection_topics.yaml
  nh_private_.param("detection_topics", detection_topics_, std::vector<std::string>(1, "detection"));
  for (size_t i = 0; i < detection_topics_.size(); i++)
  {
    sub_vector_.emplace_back(nh_.subscribe(detection_topics_[i], 1, &ObjectTrackingNode::detectionCallback, this));
  }

  // rviz pose estimate - used to test particle filter behavior without measurements
  sub_initial_pose_ = nh_.subscribe("initialpose", 1, &ObjectTrackingNode::initialPoseCallback, this);

  pub_detection_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("tracked_objects", 100);
  pub_detection_immature_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("immature_tracks", 100);
  //pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>("particles", 100);
  pub_particles_ = nh_.advertise<visualization_msgs::MarkerArray>("particles", 100);
  pub_gridmap_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 100);
  pub_cluster_centroids_ = nh_.advertise<tuw_object_msgs::ObjectDetection>("cluster_centroids", 100);

  tf_listener_ = std::make_shared<tf::TransformListener>();

  nh_private_.param("common_frame", common_frame_, std::string("map"));
  nh_private_.param("grid_map_bagfile", grid_map_bagfile_, std::string("heatmap_bagfile"));

  try
  {
    // read heat map from bag file
    ROS_INFO("reading heat map from bag file");
    rosbag::Bag bag;
    bag.open(grid_map_bagfile_, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery("heatmap_grid_map"));

    foreach (rosbag::MessageInstance const m, view)
    {
      grid_map::GridMapRosConverter::fromMessage(*m.instantiate<grid_map_msgs::GridMap>(), heat_map_);
    }
    bag.close();

    layer_ = &heat_map_["sum"];

    // publish heat map
    grid_map_msgs::GridMap message;
    heat_map_.setTimestamp(ros::Time::now().toNSec());
    grid_map::GridMapRosConverter::toMessage(heat_map_, message);
    pub_gridmap_.publish(message);
  }
  catch (rosbag::BagIOException e)
  {
    heat_map_.add("likelihood_field", 0.0);
    heat_map_.add("sum", 0.0);
    heat_map_.setFrameId("map");
    heat_map_.setGeometry(grid_map::Length(5.0, 5.0), 0.01, grid_map::Position::Zero());
  }

  object_tracker_ = std::make_unique<ObjectTrackerSeparate>(pf_config, t_config);
  
  reconfigureFnc_ = boost::bind(&ObjectTrackingNode::callbackParameters, this, _1, _2);
  reconfigureServer_.setCallback(reconfigureFnc_);
}

void ObjectTrackingNode::callbackParameters(tuw_object_tracking::tuw_object_trackingConfig& config, uint32_t level)
{
  config_ = config;

  switch(config.grid_map_layer)
  {
    case 0:
      layer_ = &heat_map_["sum"];
      break;
    case 1:
      layer_ = &heat_map_["likelihood_field"];
      break;
  }
  
  switch (config.motion_model)
  {
    case 0:  // constant velocity
      object_tracker_->pf_config_->system_model = std::make_shared<ConstVelSystemModel>(config.sigma_x_sys, config.sigma_y_sys);
      std::cout << "switch to const vel model" << std::endl;
      break;
    case 2:  // heatmap
      object_tracker_->pf_config_->system_model =
          std::make_shared<HeatMapSystemModel>(config.sigma_x_sys, config.sigma_y_sys, config.sigma_omega_sys, config.angle_partitions, config.gamma, heat_map_, layer_);
      object_tracker_->pf_config_->meas_model_inv = std::make_shared<MahalanobisMeasModelInv>(config.cov_scale_meas);
      std::cout << "switch to heatmap model" << std::endl;
      break;
    case 3:  // constant acceleration
      object_tracker_->pf_config_->system_model = std::make_shared<ConstAccSystemModel>(config.sigma_x_sys, config.sigma_y_sys);
      std::cout << "switch to constant acc model" << std::endl;
      break;
    case 4:  // coordinated turn
      object_tracker_->pf_config_->system_model =
          std::make_shared<CoordinatedTurnSystemModel>(config.sigma_x_sys, config.sigma_y_sys, config.sigma_omega_sys);
      std::cout << "switch to coordinated turn model" << std::endl;
      break;
  }
  
  switch (config.measurement_model)
  {
    case 0: // simple eucl. distance measurement model
      object_tracker_->pf_config_->meas_model = std::make_shared<SimpleMeasModel>(config.sigma_meas);
      break;
    case 1: // mahalanobis distance measurement model
      object_tracker_->pf_config_->meas_model = std::make_shared<MahalanobisMeasModel>(config.cov_scale_meas);
      object_tracker_->pf_config_->system_model_inv = std::make_shared<HeatMapSystemModelInv>(config.sigma_omega_sys, config.angle_partitions, heat_map_, layer_);
      break;
  }

  object_tracker_->pf_config_->num_particles = config.num_particles;
  object_tracker_->pf_config_->num_particles = config.num_particles;
  object_tracker_->pf_config_->resample_rate = config.resample_rate;
  object_tracker_->pf_config_->sigma_init_state = config.sigma_init_state;

  object_tracker_->pf_config_->const_fwd_pred = config.const_fwd_pred;
  object_tracker_->pf_config_->fwd_pred_time = config.fwd_pred_time;
  object_tracker_->pf_config_->particle_filter_output_modality = config.particle_filter_output_modality;
  object_tracker_->pf_config_->enable_clustering = config.enable_clustering;

  object_tracker_->t_config_->deletion_cycles = config.deletion_cycles;
  object_tracker_->t_config_->deletion_cycles_inv = config.deletion_cycles_inv;
  object_tracker_->t_config_->promotion_cycles = config.promotion_cycles;
  object_tracker_->t_config_->max_dist_for_association = config.max_dist_for_association;
  object_tracker_->t_config_->visually_confirmed = config.visually_confirmed;
  object_tracker_->t_config_->use_mahalanobis = config.use_mahalanobis;
  object_tracker_->t_config_->use_particle_mahalanobis = config.use_particle_mahalanobis;
  object_tracker_->t_config_->visual_confirmation_inc = config.visual_confirmation_inc;

  // update existing tracks
  object_tracker_->updatePFConfig();
  /*
  for (auto& i : object_tracker_->getTracks())
  {
    i.second.updateConfig(object_tracker_->pf_config_);
  }
  */
}

void ObjectTrackingNode::detectionCallback(const tuw_object_msgs::ObjectDetection& detection)
{
  tf::StampedTransform detection2common;
  try
  {
    tf_listener_->lookupTransform(common_frame_, detection.header.frame_id, ros::Time(0), detection2common);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("detectionCallback: %s", ex.what());
    return;
  }

  // create tuw::MeasurementObject, convert ros msg to measurement data structure
  tuw::MeasurementObjectPtr meas = std::make_shared<MeasurementObject>();

  // also create empty measurement if required
  meas->stamp() = detection.header.stamp.toBoost();

  meas->range_min() = detection.distance_min;
  meas->range_max() = detection.distance_max;
  meas->range_max_id() = detection.distance_max_id;

  meas->view_direction().w() = detection.view_direction.w;
  meas->view_direction().x() = detection.view_direction.x;
  meas->view_direction().y() = detection.view_direction.y;
  meas->view_direction().z() = detection.view_direction.z;

  meas->fov_horizontal() = detection.fov_horizontal;
  meas->fov_vertical() = detection.fov_vertical;

  meas->type() = detection.type;
  meas->sensor_type() = detection.sensor_type;

  meas->resize(detection.objects.size());

  for (size_t i = 0; i < meas->size(); i++)
  {
    tf::Transform p_as_tf;
    tf::poseMsgToTF(detection.objects[i].object.pose, p_as_tf);
    p_as_tf = detection2common * p_as_tf;

    geometry_msgs::PoseWithCovariance p;
    tf::poseTFToMsg(p_as_tf, p.pose);
    
    (*meas)[i].stamp = detection.header.stamp.toBoost();
    (*meas)[i].pose2d.set_x(p.pose.position.x);
    (*meas)[i].pose2d.set_y(p.pose.position.y);
    (*meas)[i].ids = detection.objects[i].object.ids;
    (*meas)[i].ids_confidence = detection.objects[i].object.ids_confidence;
    
    if(detection.objects[i].object.shape_variables.size() >= 6)
    {
      // also apply coordinate transform
      geometry_msgs::PoseWithCovariance P1;
      geometry_msgs::PoseWithCovariance P2;
      
      P1.pose.position.x = detection.objects[i].object.shape_variables[0];
      P1.pose.position.y = detection.objects[i].object.shape_variables[1];
      P1.pose.position.z = detection.objects[i].object.shape_variables[2];
      
      P2.pose.position.x = detection.objects[i].object.shape_variables[3];
      P2.pose.position.y = detection.objects[i].object.shape_variables[4];
      P2.pose.position.z = detection.objects[i].object.shape_variables[5];
      
      tf::Transform P1_as_tf;
      tf::Transform P2_as_tf;
      
      tf::poseMsgToTF(P1.pose, P1_as_tf);
      tf::poseMsgToTF(P2.pose, P2_as_tf);
      
      P1_as_tf = detection2common * P1_as_tf;
      P2_as_tf = detection2common * P2_as_tf;
      
      tf::poseTFToMsg(P1_as_tf, P1.pose);
      tf::poseTFToMsg(P2_as_tf, P2.pose);
      
      (*meas)[i].shape_variables.emplace_back(P1.pose.position.x);
      (*meas)[i].shape_variables.emplace_back(P1.pose.position.y);
      (*meas)[i].shape_variables.emplace_back(P1.pose.position.z);
      
      (*meas)[i].shape_variables.emplace_back(P2.pose.position.x);
      (*meas)[i].shape_variables.emplace_back(P2.pose.position.y);
      (*meas)[i].shape_variables.emplace_back(P2.pose.position.z);
    }

    // transform covariance C' = R C R^T
    tf::Matrix3x3 pose_cov;

    // we only need x,y,z
    pose_cov[0][0] = detection.objects[i].covariance_pose[0];
    pose_cov[0][1] = detection.objects[i].covariance_pose[1];
    pose_cov[0][2] = detection.objects[i].covariance_pose[2];
    pose_cov[1][0] = detection.objects[i].covariance_pose[3];
    pose_cov[1][1] = detection.objects[i].covariance_pose[4];
    pose_cov[1][2] = detection.objects[i].covariance_pose[5];
    pose_cov[2][0] = detection.objects[i].covariance_pose[6];
    pose_cov[2][1] = detection.objects[i].covariance_pose[7];
    pose_cov[2][2] = detection.objects[i].covariance_pose[8];

    pose_cov = detection2common.getBasis() * pose_cov * detection2common.getBasis().transpose();

    // both matrices are row major
    (*meas)[i].covariance(0) = pose_cov[0][0];
    (*meas)[i].covariance(1) = pose_cov[0][1];
    (*meas)[i].covariance(2) = pose_cov[0][2];
    (*meas)[i].covariance(3) = pose_cov[1][0];
    (*meas)[i].covariance(4) = pose_cov[1][1];
    (*meas)[i].covariance(5) = pose_cov[1][2];
    (*meas)[i].covariance(6) = pose_cov[2][0];
    (*meas)[i].covariance(7) = pose_cov[2][1];
    (*meas)[i].covariance(8) = pose_cov[2][2];
  }

  object_tracker_->addDetection(meas);
  
  //publishTracks(true);
}

void ObjectTrackingNode::printTracks(bool particles) const
{
  // output current particles for every track
  for (auto it = object_tracker_->getTracks().begin(); it != object_tracker_->getTracks().end(); it++)
  {
    cout << "tracks[" << it->first - object_tracker_->getTracks().begin()->first << "]" << endl;
    for (Eigen::Index i = 0; i < it->second.estimatedState().size(); i++)
    {
      cout << it->second.estimatedState()(i) << ", ";
    }
    cout << endl;

    // also print every particle
    if (particles)
    {
      for (size_t j = 0; j < it->second.particles()->size(); j++)
      {
        cout << "particles[" << j << "]" << endl;
        for (Eigen::Index k = 0; k < it->second.particles()->operator[](j).state.size(); k++)
        {
          cout << it->second.particles()->operator[](j).state(k) << ", ";
        }
        cout << endl;
        cout << "weight = " << it->second.particles()->operator[](j).weight << endl;
      }
    }
  }
}

void ObjectTrackingNode::publishTracks(bool particles) const
{
  // publish tracks as tuw_object_msgs::ObjectDetection
  tuw_object_msgs::ObjectDetection detection;
  detection.header.stamp = ros::Time::now();
  detection.header.frame_id = common_frame_;
  detection.type = tuw_object_msgs::ObjectDetection::OBJECT_TYPE_PERSON;
  detection.view_direction.x = 0;
  detection.view_direction.y = 0;
  detection.view_direction.z = 0;
  detection.view_direction.w = 1;
  
  tuw_object_msgs::ObjectDetection detection_immature;
  detection_immature.header.stamp = ros::Time::now();
  detection_immature.header.frame_id = common_frame_;
  detection_immature.type = tuw_object_msgs::ObjectDetection::OBJECT_TYPE_PERSON;
  detection_immature.view_direction.x = 0;
  detection_immature.view_direction.y = 0;
  detection_immature.view_direction.z = 0;
  detection_immature.view_direction.w = 1;
  
  tuw_object_msgs::ObjectDetection cluster_detection;
  cluster_detection.header.stamp = ros::Time::now();
  cluster_detection.header.frame_id = common_frame_;
  cluster_detection.type = tuw_object_msgs::ObjectDetection::OBJECT_TYPE_PERSON;
  cluster_detection.view_direction.x = 0;
  cluster_detection.view_direction.y = 0;
  cluster_detection.view_direction.z = 0;
  cluster_detection.view_direction.w = 1;

//  geometry_msgs::PoseArray pose_array;
//  pose_array.header.stamp = ros::Time::now();
//  pose_array.header.frame_id = common_frame_;

  visualization_msgs::MarkerArray marker_array;
  
  for (auto it = object_tracker_->getTracks().begin(); it != object_tracker_->getTracks().end(); it++)
  {
    tuw_object_msgs::ObjectWithCovariance obj;
    Eigen::MatrixXd estimated_state = it->second.estimatedState();
    obj.object.pose.position.x = estimated_state(0);
    obj.object.pose.position.y = estimated_state(1);
    obj.object.pose.position.z = 0;  // person is on the ground
    obj.object.pose.orientation.x = 0;
    obj.object.pose.orientation.z = 0;
    obj.object.pose.orientation.y = 0;
    obj.object.pose.orientation.w = 1;

    obj.object.twist.linear.x = estimated_state(2);
    obj.object.twist.linear.y = estimated_state(3);
    obj.object.twist.linear.z = 0;

    obj.object.ids.emplace_back(it->second.getTrackId());
    obj.object.ids_confidence.emplace_back(1.0);

    obj.covariance_pose.push_back(it->second.stateCovariance()->operator()(0, 0));
    obj.covariance_pose.push_back(it->second.stateCovariance()->operator()(0, 1));
    obj.covariance_pose.push_back(0);
    obj.covariance_pose.push_back(it->second.stateCovariance()->operator()(1, 0));
    obj.covariance_pose.push_back(it->second.stateCovariance()->operator()(1, 1));
    obj.covariance_pose.push_back(0);
    obj.covariance_pose.push_back(0);
    obj.covariance_pose.push_back(0);
    obj.covariance_pose.push_back(1);

    obj.covariance_twist.push_back(it->second.stateCovariance()->operator()(2, 2));
    obj.covariance_twist.push_back(it->second.stateCovariance()->operator()(2, 3));
    obj.covariance_twist.push_back(0);
    obj.covariance_twist.push_back(it->second.stateCovariance()->operator()(3, 2));
    obj.covariance_twist.push_back(it->second.stateCovariance()->operator()(3, 3));
    obj.covariance_twist.push_back(0);
    obj.covariance_twist.push_back(0);
    obj.covariance_twist.push_back(0);
    obj.covariance_twist.push_back(1);
      
    if (it->second.getVisibility() && (!config_.visually_confirmed || it->second.getVisualConfirumation()))
    {
      detection.objects.emplace_back(obj);
      
      // also publish particles as geometry_msgs::PoseArray
      if (particles)
      {
        auto particles = it->second.particles();
        for (size_t j = 0; j < particles->size(); j++)
        {
          /*
          geometry_msgs::Pose pose;
          pose.position.x = particles->operator[](j).state(static_cast<int>(State::X));
          pose.position.y = particles->operator[](j).state(static_cast<int>(State::Y));
          pose.position.z = 0;
          double yaw = atan2(particles->operator[](j).state(3), particles->operator[](j).state(2));
          tf::Quaternion q;
          q.setRPY(0, 0, yaw);
          pose.orientation.x = q.x();
          pose.orientation.y = q.y();
          pose.orientation.z = q.z();
          pose.orientation.w = q.w();

          pose_array.poses.emplace_back(pose);
          */

          visualization_msgs::Marker marker;
          marker.header.frame_id = common_frame_;
          marker.header.stamp = ros::Time();
          marker.ns = "tracking";
          marker.id = it->first * particles->size() + j;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.lifetime = ros::Duration(0.1);
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = particles->operator[](j).state(static_cast<int>(State::X));
          marker.pose.position.y = particles->operator[](j).state(static_cast<int>(State::Y));
          marker.pose.position.z = 0;

          double state_vx = particles->operator[](j).state(static_cast<int>(State::VX));
          double state_vy = particles->operator[](j).state(static_cast<int>(State::VY));

          double yaw = atan2(state_vy, state_vx);
          tf::Quaternion q;
          q.setRPY(0, 0, yaw);
          marker.pose.orientation.x = q.x();
          marker.pose.orientation.y = q.y();
          marker.pose.orientation.z = q.z();
          marker.pose.orientation.w = q.w();
          
          marker.scale.x = sqrt(state_vx * state_vx + state_vy * state_vy) * 1.5;
          marker.scale.y = 0.01;
          marker.scale.z = 0.01;

          marker.color.a = 1.0;
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;

          marker_array.markers.emplace_back(marker);
        }
      }
      
      for(size_t i = 0; i < it->second.cluster_centroids_->size(); i++)
      {
        tuw_object_msgs::ObjectWithCovariance obj;
        
        obj.object.pose.position.x = it->second.cluster_centroids_->operator[](i)(0);
        obj.object.pose.position.y = it->second.cluster_centroids_->operator[](i)(1);
        obj.object.pose.position.z = 0;  // person is on the ground
        obj.object.pose.orientation.x = 0;
        obj.object.pose.orientation.z = 0;
        obj.object.pose.orientation.y = 0;
        obj.object.pose.orientation.w = 1;

        obj.object.twist.linear.x = 0;
        obj.object.twist.linear.y = 0;
        obj.object.twist.linear.z = 0;

        obj.object.ids.emplace_back(it->second.getTrackId());
        obj.object.ids_confidence.emplace_back(1.0);

        obj.covariance_pose.push_back(it->second.stateCovariance()->operator()(0, 0));
        obj.covariance_pose.push_back(it->second.stateCovariance()->operator()(0, 1));
        obj.covariance_pose.push_back(0);
        obj.covariance_pose.push_back(it->second.stateCovariance()->operator()(1, 0));
        obj.covariance_pose.push_back(it->second.stateCovariance()->operator()(1, 1));
        obj.covariance_pose.push_back(0);
        obj.covariance_pose.push_back(0);
        obj.covariance_pose.push_back(0);
        obj.covariance_pose.push_back(1);

        obj.covariance_twist.push_back(it->second.stateCovariance()->operator()(2, 2));
        obj.covariance_twist.push_back(it->second.stateCovariance()->operator()(2, 3));
        obj.covariance_twist.push_back(0);
        obj.covariance_twist.push_back(it->second.stateCovariance()->operator()(3, 2));
        obj.covariance_twist.push_back(it->second.stateCovariance()->operator()(3, 3));
        obj.covariance_twist.push_back(0);
        obj.covariance_twist.push_back(0);
        obj.covariance_twist.push_back(0);
        obj.covariance_twist.push_back(1);

        cluster_detection.objects.emplace_back(obj);
      }
    }
    else
    {
      detection_immature.objects.emplace_back(obj);
    }

  }
  pub_detection_.publish(detection);
  pub_detection_immature_.publish(detection_immature);
  pub_cluster_centroids_.publish(cluster_detection);
  if (particles)
  {
    //pub_particles_.publish(pose_array);
    pub_particles_.publish(marker_array);
  }
  
  if (config_.print_tracks)
      printTracks(config_.print_particles);
}

void ObjectTrackingNode::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  Eigen::Matrix<double, STATE_SIZE, 1> init_state;

  // direction from pose
  tf::Pose pose_tf;
  tf::poseMsgToTF(pose.pose.pose, pose_tf);
  double yaw = tf::getYaw(pose_tf.getRotation());

  init_state(0) = pose.pose.pose.position.x;
  init_state(1) = pose.pose.pose.position.y;
  init_state(2) = 0.1 * cos(yaw);
  init_state(3) = 0.1 * sin(yaw);
  init_state(4) = 0;
  init_state(5) = 0;
  init_state(6) = 0;

  int id = objectTracker().createTrack(init_state);
  
  object_tracker_->getTracks().at(id).setVisibility(true);
  object_tracker_->getTracks().at(id).setVisualConfirmation(true);
  for(int i = 0; i < 100; i++)
    object_tracker_->getTracks().at(id).incVisualConfirmation();

  ROS_INFO("create track with initial pose: (%f, %f) with id: %d", init_state(0), init_state(1), id);

  //publishTracks(true);
  //printTracks(true);

  // publish heat map
  grid_map_msgs::GridMap message;
  heat_map_.setTimestamp(ros::Time::now().toNSec());
  grid_map::GridMapRosConverter::toMessage(heat_map_, message);
  pub_gridmap_.publish(message);
}

ObjectTracker& ObjectTrackingNode::objectTracker() const
{
  return *object_tracker_;
}


int main(int argc, char** argv)
{    
  ros::init(argc, argv, "tuw_object_tracking_node");

  ros::NodeHandle nh;

  std::shared_ptr<ParticleFilterConfig> pf_config = std::make_shared<ParticleFilterConfig>();
  std::shared_ptr<TrackerConfig> t_config = std::make_shared<TrackerConfig>();

  pf_config->num_particles = 100;
  pf_config->sigma_init_state = 1;
  pf_config->system_model = std::make_shared<ConstVelSystemModel>(1, 1);
  pf_config->meas_model = std::make_shared<SimpleMeasModel>(1);
  pf_config->resample_rate = 1.0;
  pf_config->const_fwd_pred = false;
  pf_config->fwd_pred_time = 1.0;
  pf_config->particle_filter_output_modality = 1;
  pf_config->enable_clustering = false;

  t_config->deletion_cycles = 1;
  t_config->promotion_cycles = 1;
  t_config->visual_confirmation_inc = 2;
  t_config->max_dist_for_association = 2.0;
  t_config->visually_confirmed = false;
  t_config->use_mahalanobis = false;
  t_config->use_particle_mahalanobis = false;

  ObjectTrackingNode object_tracking_node(nh, pf_config, t_config);

  ros::Rate r(60);  // 60Hz
  
  while (ros::ok())
  {
    object_tracking_node.objectTracker().predict(ros::Time::now().toBoost());
    
    object_tracking_node.objectTracker().update();

    object_tracking_node.publishTracks(true);
    
    object_tracking_node.objectTracker().clearDetections();
    
    ros::spinOnce();
    if (!r.sleep())
    {
      ROS_WARN("In %s: Loop missed desired rate of %.4fs (loop actually took %.4fs)", ros::this_node::getName().c_str(),
               r.expectedCycleTime().toSec(), r.cycleTime().toSec());
    }
  }

  return 0;
}
