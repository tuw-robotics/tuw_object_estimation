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

#include "coordinated_turn_system_model.h"
#include <assert.h>
#include <common.h>
#include <cmath>

CoordinatedTurnSystemModel::CoordinatedTurnSystemModel(double sigma_x, double sigma_y, double sigma_omega) : SystemModel(7)
{
  std::random_device rd;
  generator_ = std::mt19937(rd());
  normal_distribution_ = std::normal_distribution<double>();
  sigma_x_ = sigma_x;
  sigma_y_ = sigma_y;
  sigma_omega_ = sigma_omega;
}

void CoordinatedTurnSystemModel::sample(Eigen::Ref<Eigen::VectorXd> state, double dt)
{
  assert(state.size() >= state_size_);
  Eigen::VectorXd sample = state;
  
  if(state(static_cast<int>(State::OMEGA)) != 0.0)
  {
    sample(static_cast<int>(State::X)) = state(static_cast<int>(State::X)) + state(static_cast<int>(State::VX)) * sin(dt * state(static_cast<int>(State::OMEGA))) / state(static_cast<int>(State::OMEGA))
                    + state(static_cast<int>(State::VY)) * (cos(dt * state(static_cast<int>(State::OMEGA))) - 1) / state(static_cast<int>(State::OMEGA)) 
                    + dt * dt * 0.5 * sigma_x_ * sigma_x_ * normal_distribution_(generator_);
                    
    sample(static_cast<int>(State::Y)) = state(static_cast<int>(State::Y)) - state(static_cast<int>(State::VX)) * (cos(dt * state(static_cast<int>(State::OMEGA))) - 1) / state(static_cast<int>(State::OMEGA))
                    + state(static_cast<int>(State::VY)) * sin(dt * state(static_cast<int>(State::OMEGA))) / state(static_cast<int>(State::OMEGA)) 
                    + dt * dt * 0.5 * sigma_y_ * sigma_y_ * normal_distribution_(generator_);
                    
    sample(static_cast<int>(State::VX)) = state(static_cast<int>(State::VX)) * cos(dt * state(static_cast<int>(State::OMEGA))) 
                    - state(static_cast<int>(State::VY)) * sin(dt * state(static_cast<int>(State::OMEGA))) 
                    + dt * sigma_x_ * sigma_x_ * normal_distribution_(generator_);
                    
    sample(static_cast<int>(State::VY)) = state(static_cast<int>(State::VX)) * sin(dt * state(static_cast<int>(State::OMEGA))) 
                    + state(static_cast<int>(State::VY)) * cos(dt * state(static_cast<int>(State::OMEGA))) 
                    + dt * sigma_y_ * sigma_y_ * normal_distribution_(generator_);
                    
    sample(static_cast<int>(State::OMEGA)) = state(static_cast<int>(State::OMEGA)) + dt * sigma_omega_ * sigma_omega_ * normal_distribution_(generator_);
  }
  else  // limit
  {
    sample(static_cast<int>(State::X)) = state(static_cast<int>(State::X)) + state(static_cast<int>(State::VX)) * dt
                    + dt * dt * 0.5 * sigma_x_ * sigma_x_ * normal_distribution_(generator_);
                    
    sample(static_cast<int>(State::Y)) = state(static_cast<int>(State::Y)) + state(static_cast<int>(State::VY)) * dt 
                    + dt * dt * 0.5 * sigma_y_ * sigma_y_ * normal_distribution_(generator_);
                    
    sample(static_cast<int>(State::VX)) = state(static_cast<int>(State::VX))
                     + dt * sigma_x_ * sigma_x_ * normal_distribution_(generator_);
                    
    sample(static_cast<int>(State::VY)) = state(static_cast<int>(State::VY))
                     + dt * sigma_y_ * sigma_y_ * normal_distribution_(generator_);
                    
    sample(static_cast<int>(State::OMEGA)) = dt * sigma_omega_ * sigma_omega_ * normal_distribution_(generator_);
  }
  
  state = sample;
}

void CoordinatedTurnSystemModel::sample(Eigen::Ref<Eigen::VectorXd> state, double dt, Eigen::Ref<Eigen::Vector2d> F, double m)
{
  sample(state, dt);
}
