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

#ifndef CONST_ACC_SYSTEM_MODEL_H
#define CONST_ACC_SYSTEM_MODEL_H

#include <system_model.h>

/*!
 * This class represents the (nearly) constant acceleration motion model.
 */
class ConstAccSystemModel : public SystemModel
{
public:
  /*!
   * Constructor
   * 
   * @param sigma_x Sigma for acceleration noise in x-direction
   * @param sigma_y Sigma for acceleration noise in y-direction
   */
  ConstAccSystemModel(double sigma_x, double sigma_y);

  /*!
   * Forward predicts a particle assuming nearly (noisy) constant acceleration
   * 
   * @param state Current state of the tracked object / returns next state
   * @param dt Forward prediction time
   * @param meas (optional) Current measurement if used in forward prediction
   * @param meas_cov (optional) Corresponding measurement covariance
   */
  void sample(Eigen::Ref<Eigen::VectorXd> state, double dt, const Eigen::Ref<const Eigen::VectorXd>& meas, const Eigen::Ref<const Eigen::MatrixXd>& meas_cov) override;

private:
  std::mt19937 generator_;                                /// random number generator
  std::normal_distribution<double> normal_distribution_;  /// normal distribution for generic use
  double sigma_x_;
  double sigma_y_;
};

#endif  // CONST_ACC_SYSTEM_MODEL_H
