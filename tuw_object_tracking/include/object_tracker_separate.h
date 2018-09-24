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

#ifndef OBJECT_TRACKER_SEPARATE_H
#define OBJECT_TRACKER_SEPARATE_H

#include "object_tracker.h"
#include <tracker_config.h>
#include <track.h>
#include <common.h>

/*!
 * This class implements a tracking algorithm using separate particle filters for each object to track.
 */
class ObjectTrackerSeparate : public ObjectTracker
{
public:
  ObjectTrackerSeparate(std::shared_ptr<ParticleFilterConfig> pf_config, std::shared_ptr<TrackerConfig> t_config);
  void update() override;
private:
  /*!
   * Defines best assignments between tracks and detections using the Kuhn-Munkres algorithm.
   * 
   * @param detection Pointer to the detections that should be matched
   * @param assignment Array which holds track <-> detection assignments track 1 is assigned to measurement assignment[1]
   * @param D Cost matrix defined by a distance measure between tracks and detections (i.e. Mahalanobis distance)
   * 
   * @return Returns true if sucessfully assigned with detection size > 0
   */
  bool calcAssignments(const MeasurementObjectConstPtr& detection, std::vector<int>& assignment, Eigen::MatrixXd& D);
};

#endif  // OBJECT_TRACKER_SEPARATE_H
