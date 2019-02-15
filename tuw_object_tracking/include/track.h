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

#ifndef TRACK_H
#define TRACK_H

#include <particle_filter_config.h>
#include <particle_filter.h>
#include <tuw_geometry/tuw_geometry.h>

/*!
 * @brief Template class representing an object's track over time.
 *        Directly derived from the particle filter.
 * 
 * @tparam stateDim Dimension of the tracked object's state.
 */
template <int stateDim>
class Track : public ParticleFilter<stateDim>
{
public:
  /*!
   * Constructor
   *
   * @param init_state Initial state of the track.
   * @param config The underlying particle filter config.
   * @param id The unique track ID.
   * @param visual_confirmation_inc Value of which the visual confirmation counter is increased if a visual detection occurs.
   */
  Track(const Eigen::Matrix<double, stateDim, 1>& init_state, std::shared_ptr<ParticleFilterConfig> config, const int id, const int visual_confirmation_inc)
    : ParticleFilter<stateDim>(config), track_id_(id), visual_confirmation_inc_(visual_confirmation_inc)
  {
    ParticleFilter<stateDim>::initAroundState(init_state, config->sigma_init_state);
    delete_ = 0;
    promote_ = 0;
    visible_ = false;
    visually_confirmed_ = false;
    visual_confirmation_ = 1;
  }

  /*!
   * Getter for the track id
   *
   * @return track id
   */
  const int getTrackId() const
  {
    return track_id_;
  }

  /*!
   * reset the delete counter
   */
  void resetDelete()
  {
    delete_ = 0;
  }

  /*!
   * increment the delete counter by 1
   */
  void incDelete()
  {
    delete_++;
  }

  /*!
   * Getter for the delete counter
   *
   * @return delete counter
   */
  int getDelete() const
  {
    return delete_;
  }

  /*!
   * set track visibility
   *
   * @param visible Boolean value whether the track should be visible or not.
   */
  void setVisibility(bool visible)
  {
    visible_ = visible;
  }

  /*!
   * Getter for visibility
   */
  bool getVisibility() const
  {
    return visible_;
  }

  /*!
   * Increment maturity promotion counter by 1.
   */
  void incPromote()
  {
    promote_++;
  }

  /*!
   * Getter for maturity promotion counter.
   *
   * @return promotion counter value
   */
  int getPromote() const
  {
    return promote_;
  }

  /*!
   * Set visual confirmation
   *
   * @param Boolean value whether the track is visually confirmed or not.
   */
  void setVisualConfirmation(bool visually_confirmed)
  {
    visually_confirmed_ = visually_confirmed;
  }

  /*!
   * Getter for visual confirmation
   *
   * @return Boolean value whether the track is visually confirmed or not.
   */
  bool getVisualConfirumation() const
  {
    return visual_confirmation_ > 0;
    //return visually_confirmed_;
  }

  /*!
   * Increment visual confirmation counter by the value set in the constructor.
   * This also enables particle clustering for visible tracks. However clustering
   * will not be performed unless it is set in the particle filter config too.
   */
  void incVisualConfirmation()
  {
    visual_confirmation_ += visual_confirmation_inc_;
    this->setEnableClustering(visual_confirmation_ > 0 && visible_);
  }

  /*!
   * Decrement visual confirmation by 1.
   */
  void decVisualConfirmation()
  {
    visual_confirmation_--;
  }

  /*!
   * Get visual confirmation counter.
   */
  int getVisualConfirmationCount()
  {
    return visual_confirmation_;
  }

private:
  int track_id_;
  int delete_;
  int promote_;
  int visual_confirmation_;
  const int visual_confirmation_inc_;
  bool visible_;
  bool visually_confirmed_;
};
#endif  // TRACK_H
