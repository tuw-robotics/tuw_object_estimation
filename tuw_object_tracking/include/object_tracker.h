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

#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include <tuw_object_msgs/ObjectDetection.h>
#include <tuw_geometry/tuw_geometry.h>
#include <measurement_object.h>
#include <particle_filter_config.h>
#include <tracker_config.h>
#include <track.h>
#include <common.h>

/*!
 * Generic object tracker class. Contains the abstract method update which
 * has to be implemented by specific derived tracking classes.
 */
class ObjectTracker
{
public:
  ObjectTracker(std::shared_ptr<ParticleFilterConfig> pf_config, std::shared_ptr<TrackerConfig> t_config);
  virtual ~ObjectTracker() {};
  
  /*!
   * Creates a new track based on an initial state.
   * 
   * @param init_state Initial state of the track.
   * 
   * @return Retruns the id of the newly created track.
   */
  int createTrack(const Eigen::Matrix<double, STATE_SIZE, 1> init_state);
  
  /*!
   * Deletes a track by id.
   * 
   * @param track_id ID of the track that shell be deleted
   */
  void deleteTrack(const int track_id);
  
  /*!
   * Getter method for tracks.
   * 
   * @return Returns a reference of the track std::map
   */
  std::map<int, Track<STATE_SIZE>>& getTracks();
  
  /*!
   * Adds received detections to the tracker
   * 
   * @param detection Pointer to the received detection.
   */
  void addDetection(const MeasurementObjectConstPtr& detection);
  
  /*!
   * Forward prediction for all tracks
   * 
   * @param current_time_stamp Current time, up to which the tracks should be predicted
   */
  void predict(boost::posix_time::ptime current_time_stamp);
  
  /*!
   * Deletes all detections in the tracker
   */
  void clearDetections();
  
  /*!
   * Sets the configuration struct of the particle filters
   * 
   * @param pf_config Particle filter configuration struct
   */
  void setPFConfig(ParticleFilterConfig& pf_config);
  
  /*!
   * Updates all particle filters (tracks) with the currently set config.
   */
  void updatePFConfig();
  
  /*!
   * Sets the configuration struct of the tracker
   * 
   * @param t_config Tracker configuration struct
   */
  void setTrackerConfig(TrackerConfig& t_config);
  
  /*!
   * Updates all active tracks with the currently set config.
   */
  void updateTrackerConfig();
  
  /*!
   * Updates tracker according to received measurements. Handles track creation / deletion,
   * data association and forward prediction. Needs to be implemented by a derived tracker.
   */
  virtual void update() = 0;
  
  std::shared_ptr<ParticleFilterConfig> pf_config_;
  std::shared_ptr<TrackerConfig> t_config_;
protected:
  int track_id_counter_;
  std::vector<MeasurementObjectConstPtr> detections_;
  std::map<int, Track<STATE_SIZE>> tracks_;
};

#endif  // OBJECT_TRACKER_H
