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

#ifndef SYSTEM_MODEL_H
#define SYSTEM_MODEL_H

#include <eigen3/Eigen/Dense>
#include <random>

/*!
 * This abstract class represents a generic system model.
 */
class SystemModel
{
public:
  SystemModel(unsigned int state_size) : state_size_(state_size) {}
  virtual ~SystemModel() {}

  /*!
   * Forward predicts a single particle from its current state to the next state.
   * Needs to be implemented by deriving classes using a particular motion model.
   * 
   * @param state Current state of the tracked object / returns next state
   * @param dt Forward prediction time
   * @param meas (optional) Current measurement if used in forward prediction
   * @param meas_cov (optional) Corresponding measurement covariance
   */
  virtual void sample(Eigen::Ref<Eigen::VectorXd> state, double dt, const Eigen::Ref<const Eigen::VectorXd>& meas = Eigen::VectorXd(), const Eigen::Ref<const Eigen::MatrixXd>& meas_cov = Eigen::MatrixXd()) = 0;
  
  unsigned int getStateSize() const
  {
    return state_size_;
  }
protected:
  const unsigned int state_size_;
};

#endif  // SYSTEM_MODEL_H
