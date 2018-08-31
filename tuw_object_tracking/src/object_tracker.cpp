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
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/date_time/posix_time/time_formatters.hpp>

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

std::map<int, Track<STATE_SIZE>>& ObjectTracker::getTracks() 
{
  return tracks_;
}

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

void ObjectTracker::updatePFConfig()
{
  for (auto& i : tracks_)
  {
    i.second.updateConfig(pf_config_);
  }

}

