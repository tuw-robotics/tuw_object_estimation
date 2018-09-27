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

#ifndef MAHALANOBIS_MEAS_MODEL_H
#define MAHALANOBIS_MEAS_MODEL_H

#include <meas_model.h>
#include <common.h>

/*!
 * This class represents a measurement model based on the mahalanobis distance.
 */
class MahalanobisMeasModel : public MeasModel
{
public:
  /*!
   * Constructor
   * @param cov_scale Scale factor for the measurement covariance
   */
  MahalanobisMeasModel(double cov_scale);

/*!
 * Returns the probability of a measurement given the current particle state using the
 * Mahalanobis distance, which is chi-square distributed.
 * 
 * @param curr_state Current state of the particle
 * @param meas Received measurement
 * @param meas_cov Corresponding covariance Matrix
 * @param dt (optional) timestep, used to predict particles according to measurement model
 * 
 * @return probability of meas given curr_state
 */
  double getProbability(const Ref<const VectorXd>& curr_state, const Ref<const VectorXd>& meas, const Ref<const MatrixXd>& meas_cov, double dt = 0) override;
private:
  double cov_scale_;
  static const int state_dim_ = 4;
  static const int meas_dim_ = 2;
};

#endif  // MAHALANOBIS_MEAS_MODEL_H
