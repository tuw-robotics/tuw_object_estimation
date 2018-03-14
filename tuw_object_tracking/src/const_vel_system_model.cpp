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

#include "const_vel_system_model.h"
#include <assert.h>
#include <common.h>

ConstVelSystemModel::ConstVelSystemModel(double sigma_x, double sigma_y) : SystemModel(4)
{
  std::random_device rd;
  generator_ = std::mt19937(rd());
  normal_distribution_ = std::normal_distribution<double>();
  sigma_x_ = sigma_x;
  sigma_y_ = sigma_y;
}

void ConstVelSystemModel::sample(Eigen::Ref<Eigen::VectorXd> state, double dt)
{
  // motion model (constant velocity)
  // state = [x, y, vx, vy]^T
  // x(t) = x(t-1) + vx(t-1)*dt
  // y(t) = y(t-1) + vy(t-1)*dt
  // vx(t) = vx(t-1);
  // vy(t) = vy(t-1);
  // B = 0, input for object movement is unknown
  // noise model is based on unknown input in form of acceleration
  // with n ~ G*N(0, sigma_a^2)
  // a = [ax, ay, 0, 0]
  // G = [dt^2/2, 0, 0, 0; 0, dt^2/2, 0, 0; 0, 0, dt, 0; 0, 0, 0, dt]
  
  // check param dimensions, maybe dont assert and use return instead?
  assert(state.size() >= state_size_);
  Eigen::VectorXd sample = state;
  
  sample(STATE_X) = state(STATE_X) + state(STATE_VX) * dt
                    + dt * dt * 0.5 * sigma_x_ * sigma_x_ * normal_distribution_(generator_);
                    
  sample(STATE_Y) = state(STATE_Y) + state(STATE_VY) * dt 
                  + dt * dt * 0.5 * sigma_y_ * sigma_y_ * normal_distribution_(generator_);
                  
  sample(STATE_VX) = state(STATE_VX)
                    + dt * sigma_x_ * sigma_x_ * normal_distribution_(generator_);
                  
  sample(STATE_VY) = state(STATE_VY)
                    + dt * sigma_y_ * sigma_y_ * normal_distribution_(generator_);

  state = sample;
}

void ConstVelSystemModel::sample(Eigen::Ref<Eigen::VectorXd> state, double dt, Eigen::Ref<Eigen::Vector2d> F, double m)
{
  sample(state, dt);
}
