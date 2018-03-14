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

#include "mahalanobis_meas_model.h"
#include <boost/math/distributions/chi_squared.hpp>
#include <assert.h>

MahalanobisMeasModel::MahalanobisMeasModel(double cov_scale)
{
  cov_scale_ = cov_scale;
}

double MahalanobisMeasModel::getProbability(const Ref<const VectorXd>& curr_state, const Ref<const VectorXd>& meas, const Ref<const MatrixXd>& meas_cov)
{
  // simple measurement model, position directly observable (by detector)
  // state = [x, y, vx, vy]^T
  // z = [x, y]^T
  // C = [1, 0, 0, 0; 0, 1, 0, 0]
  
  // calculate mahalanobis distance from predicted measurement to actual measurement
  // note: consider current state as deterministic state with no covariance
  // only measurement provides covariance
  // mahalanobis distance is chi_square distributed
  
  double dist = sqrt((curr_state.block<2, 1>(0, 0) - meas).transpose() * (meas_cov.block<2, 2>(0, 0).inverse() * cov_scale_) * (curr_state.block<2, 1>(0, 0) - meas));
  
  //boost::math::normal norm = boost::math::normal(0, sigma_ * sigma_);
  boost::math::chi_squared chi(2);

  return boost::math::pdf(chi, dist);
}
