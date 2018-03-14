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

#include "object_tracker.h"
#include <limits>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include "munkres.h"

ObjectTracker::ObjectTracker(std::shared_ptr<ParticleFilterConfig> pf_config, std::shared_ptr<TrackerConfig> t_config)
{
  pf_config_ = pf_config;
  t_config_ = t_config;
  track_id_counter_ = 0;
}

int ObjectTracker::createTrack(const Eigen::Matrix<double, STATE_SIZE, 1> init_state)
{
  tracks_.emplace(std::piecewise_construct, std::forward_as_tuple(track_id_counter_),
                  std::forward_as_tuple(init_state, pf_config_, track_id_counter_, t_config_->visual_confirmation_inc));
  return track_id_counter_++;
}

void ObjectTracker::deleteTrack(const int track_id)
{
  tracks_.erase(track_id);
}

// data association, Nearest Neighbor
// create new tracks if required,
// delete unnecessary tracks if required
void ObjectTracker::addDetection(const MeasurementObjectConstPtr& detection)
{  
  detections_.emplace_back(detection);
}

void ObjectTracker::clearDetections()
{
  detections_.clear();
}

void ObjectTracker::predict(boost::posix_time::ptime current_time_stamp)
{
  for(auto&& track: tracks_)
  {
    track.second.predict(current_time_stamp);
  }
}

bool ObjectTracker::calcAssignments(const MeasurementObjectConstPtr& detection, std::vector<int>& assignment, Eigen::MatrixXd& D)
{
  if(detection->size() == 0)
  {
    return false;
  }

  int n = tracks_.size() > detection->size() ? tracks_.size() : detection->size();

  D.resize(n, n);
  
  int track_idx = 0;
  size_t detection_idx = 0;
  
  for(auto&& track: tracks_)
  {      
    for(detection_idx = 0; detection_idx < (*detection).size(); detection_idx++)
    {     
      Eigen::Matrix<double, 2, 2> S = (*detection)[detection_idx].covariance.block<2, 2>(0, 0);
      S = S + track.second.stateCovariance()->block<2, 2>(0, 0);
      
      Eigen::Matrix<double, 2, 1> detection_position;
      detection_position(0, 0) = (*detection)[detection_idx].pose2d.x();
      detection_position(1, 0) = (*detection)[detection_idx].pose2d.y();
      
      Eigen::Matrix<double, 2, 1> track_position = track.second.estimatedState().block<2, 1>(0, 0);
      
      if(t_config_->use_mahalanobis && detection->sensor_type() != SENSOR_TYPE_GENERIC_MONOCULAR_VISION)
      {
        // calculate mahalanobis distance between detection and track
        D(track_idx, detection_idx) = sqrt((detection_position - track_position).transpose() * S.inverse() * (detection_position - track_position));
      }
      else if(detection->sensor_type() == SENSOR_TYPE_GENERIC_MONOCULAR_VISION)
      {
        Eigen::Vector2d P1;
        Eigen::Vector2d P2;
        Eigen::Vector2d P0;
        
        if((*detection)[detection_idx].shape_variables.size() >= 6)
        {
          P1 << (*detection)[detection_idx].shape_variables[0], (*detection)[detection_idx].shape_variables[1], (*detection);
          
          P2 << (*detection)[detection_idx].shape_variables[3], (*detection)[detection_idx].shape_variables[4], (*detection);
          
          P0 = track.second.estimatedState().head(2); // first two elements of state, i.e. x and y
          
          D(track_idx, detection_idx) = acos((P2 - P1).normalized().dot((P0 - P1).normalized()));
        }
        else
        {
          std::cout << "error, shape variables not present" << std::endl;
          return false;
        }
      }
      else
      {
        D(track_idx, detection_idx) = (detection_position - track_position).norm();
      }
    }
    track_idx++;
  }
  
  // augment matrix to square size
  for(int i = track_idx; i < n; i++)
  {
    for(int j = 0; j < n; j++)
    {
      D(i, j) = 0;
    }
  }
  for(int i = detection_idx; i < n; i++)
  {
    for(int j = 0; j < n; j++)
    {
      D(j, i) = 0;
    }
  }
  
  // elements mean assign track 1 to meas assignment[1] etc.
  // hungarian algorithm / munkres algorithm O(n^3)
  std::vector<std::pair<int, int>> assignment_pairs = tuw::Munkres::find_minimum_assignment(D);
  
  assignment.resize(n);
  
  for(auto&& a = assignment_pairs.begin(); a != assignment_pairs.end(); a++)
  {
    assignment.at(a->first) = a->second;
  }
  
  return true;
}

void ObjectTracker::update()
{  
  if(detections_.empty())
  {
    MeasurementObject::Object empty_dummy;
    
    // deleting from map while looping is not a good idea ...
    auto tmp_tracks = tracks_;
    
    for(auto&& track_it: tmp_tracks)
    {
      // use .at here since [] creates an element if it does not exist
      if((track_it.second.getVisibility() && track_it.second.getDelete() > t_config_->deletion_cycles) || 
        (!track_it.second.getVisibility() && track_it.second.getDelete() > t_config_->deletion_cycles_inv))
      {
        tracks_.erase(track_it.first);
      }
      else
      {
        tracks_.at(track_it.first).update(empty_dummy, true);
        tracks_.at(track_it.first).incDelete();
      }
    }
  }
  
  for(auto&& det: detections_)
  {
    Eigen::MatrixXd D;
    std::vector<int> assignment;
    
    bool assigned = calcAssignments(det, assignment, D);

    // update tracks with measurements
    int track_idx = 0;
    std::vector<int> not_updated_tracks;
    std::vector<int> unused_detections;
    unused_detections.resize(det->size());
    for(size_t i = 0; i < det->size(); i++)
    {
      unused_detections[i] = i;
    }

    for(auto&& track_it: tracks_)
    {
      
      if(assigned && D(track_idx, assignment[track_idx]) > 0 && D(track_idx, assignment[track_idx]) < t_config_->max_dist_for_association)
      {      
        track_it.second.update((*det)[assignment[track_idx]], false);
        track_it.second.incPromote();
        track_it.second.resetDelete();
        if(det->sensor_type() == SENSOR_TYPE_GENERIC_MONOCULAR_VISION || det->sensor_type() == SENSOR_TYPE_GENERIC_RGBD)
        {
          //track_it.second.setVisualConfirmation(true);
          track_it.second.incVisualConfirmation();
        }
        else
        {
          track_it.second.decVisualConfirmation();
        }
        //std::cout << "updated track[" << track_idx << "], id = " << track_it.first << " with detection[" << assignment[track_idx] << "]" << std::endl;
        unused_detections[assignment[track_idx]] = -1;
      }
      else
      {
        not_updated_tracks.emplace_back(track_it.first);
      }
      
      // set tracks with more than 1 promotions to visible
      if(track_it.second.getPromote() > t_config_->promotion_cycles)
      {
        track_it.second.setVisibility(true);
      }
      
      track_idx++;
    }
    //std::cout << std::endl;

    // consider measurements not assigned to a track for track creation
    // if visually confirmed is active, only create if from visual detector
    for(size_t i = 0; i < unused_detections.size(); i++)
    {
      // simply create tracks for unassigned measurements
      if(unused_detections[i] != -1)
      {
        Eigen::Matrix<double, STATE_SIZE, 1> init_state;
        
        init_state(static_cast<int>(State::X)) = (*det)[unused_detections[i]].pose2d.x();
        init_state(static_cast<int>(State::Y)) = (*det)[unused_detections[i]].pose2d.y();
        init_state(static_cast<int>(State::VX)) = 0;
        init_state(static_cast<int>(State::VY)) = 0;
        init_state(static_cast<int>(State::AX)) = 0;
        init_state(static_cast<int>(State::AY)) = 0;
        init_state(static_cast<int>(State::OMEGA)) = 0;
        
        if(det->sensor_type() != SENSOR_TYPE_GENERIC_MONOCULAR_VISION)
        {
          int created_id = createTrack(init_state);
          
          if(det->sensor_type() == SENSOR_TYPE_GENERIC_RGBD)
          {
            tracks_.at(created_id).incVisualConfirmation();
          }
        }
      }
    }

    // consider tracks without measurement for deletion
    // delete if no measurement occured within a certain number of cycles
    // otherwise forward predict w/o weighting
    
    MeasurementObject::Object empty_dummy;
    empty_dummy.stamp = det->stamp();
    
    for(auto&& to_delete: not_updated_tracks)
    {
      // use .at here since [] creates an element if it does not exist
      if((tracks_.at(to_delete).getVisibility() && tracks_.at(to_delete).getDelete() > t_config_->deletion_cycles) || 
        (!tracks_.at(to_delete).getVisibility() && tracks_.at(to_delete).getDelete() > t_config_->deletion_cycles_inv))
      {
        tracks_.erase(to_delete);
      }
      else
      {
        //std::cout << "forward predict w/ dummy" << std::endl;
        tracks_.at(to_delete).update(empty_dummy, true);
        tracks_.at(to_delete).incDelete();
      }
    }
  }
} 
