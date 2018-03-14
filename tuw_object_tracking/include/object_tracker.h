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
#include <particle_filter_config.h>
#include <tracker_config.h>
#include <track.h>
#include <common.h>

class ObjectTracker
{
public:
  ObjectTracker(std::shared_ptr<ParticleFilterConfig> pf_config, std::shared_ptr<TrackerConfig> t_config);
  int createTrack(const Eigen::Matrix<double, STATE_SIZE, 1> init_state);
  void deleteTrack(const int track_id);
  void addDetection(const MeasurementObjectConstPtr& detection);
  void update();
  void predict(boost::posix_time::ptime current_time_stamp);
  void clearDetections();
protected:
  std::map<int, Track<STATE_SIZE> > tracks_;
  std::map<int, Track<STATE_SIZE> > track_candidates_;
  std::shared_ptr<ParticleFilterConfig> pf_config_;
  std::shared_ptr<TrackerConfig> t_config_;
private:
  bool calcAssignments(const MeasurementObjectConstPtr& detection, std::vector<int>& assignment, Eigen::MatrixXd& D);
  
  int track_id_counter_;
  std::vector<MeasurementObjectConstPtr> detections_;
  /*
  MeasurementObjectConstPtr detection_laser_;
  MeasurementObjectConstPtr detection_camera_rgbd_;
  MeasurementObjectConstPtr detection_camera_rgb_; */
};

#endif  // OBJECT_TRACKER_H
