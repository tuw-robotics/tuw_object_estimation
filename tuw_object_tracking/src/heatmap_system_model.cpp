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

#include "heatmap_system_model.h"
#include <assert.h>
#include <common.h>
#include <boost/math/distributions/normal.hpp>

HeatMapSystemModel::HeatMapSystemModel(double sigma_x, double sigma_y, double sigma_theta, int angle_partitions, double gamma, grid_map::GridMap& heat_map,
                                       grid_map::Matrix* layer) : SystemModel(4)
{
  std::random_device rd;
  generator_ = std::mt19937(rd());
  normal_distribution_ = std::normal_distribution<double>();
  sigma_x_ = sigma_x;
  sigma_y_ = sigma_y;
  sigma_theta_ = sigma_theta;
  heat_map_ = heat_map;
  layer_ = layer;
  angle_partitions_ = angle_partitions;
//  direction_probabilities_.resize(angle_partitions);
  direction_probabilities_.assign(angle_partitions, 1.0);
  weight_factor_.resize(angle_partitions);
  gamma_ = gamma;
  
  sum_for_weight_factor_ = 0;
  
  // precompute weight factors
  for(int i = 0; i < angle_partitions_; i++)
  {    
    double angle = atan2(sin(i * 2 * M_PI / angle_partitions_), cos(i * 2 * M_PI / angle_partitions_));
    
    boost::math::normal norm = boost::math::normal(0, sigma_theta_ * sigma_theta_);
    weight_factor_.at(i) = boost::math::pdf(norm, angle);
  }
}

void HeatMapSystemModel::sample(Eigen::Ref<Eigen::VectorXd> state, double dt, const Eigen::Ref<const Eigen::VectorXd>& meas = Eigen::VectorXd(), const Eigen::Ref<const Eigen::MatrixXd>& meas_cov = Eigen::MatrixXd())
{  
  // check param dimensions
  assert(state.size() >= state_size_);
  Eigen::VectorXd sample = state;

  // current state vel vector angle theta and magnitude r
  double theta = atan2(state(static_cast<int>(State::VY)), state(static_cast<int>(State::VX)));
  double r = sqrt(state(static_cast<int>(State::VX)) * state(static_cast<int>(State::VX)) + state(static_cast<int>(State::VY)) * state(static_cast<int>(State::VY)));

  // calculate map index from current position
  grid_map::Index idx;
  grid_map::Position pos(state(static_cast<int>(State::X)), state(static_cast<int>(State::Y)));
  double theta_tar;
  int direction;
  
  direction_probabilities_.assign(angle_partitions_, 1.0);

  if (heat_map_.getIndex(pos, idx))
  {
    bool all_zero = true;
    
    for(int i = 0; i < angle_partitions_; i++)
    {
      theta_tar = theta + i * 2 * M_PI / angle_partitions_;
      
      grid_map::Index next_idx;
      grid_map::Position next_pos = pos + grid_map::Position(r * cos(theta_tar), r * sin(theta_tar));
      
      // line iterator from idx to next_idx
      // multiply values
      
      if(heat_map_.getIndex(next_pos, next_idx))
      {
        unsigned int it_steps = 0;
        for(grid_map::LineIterator it(heat_map_, idx, next_idx); !it.isPastEnd(); ++it)
        {
          const grid_map::Index current_idx(*it);
          const size_t lin_idx = grid_map::getLinearIndexFromIndex(current_idx, heat_map_.getSize());
          
          direction_probabilities_[i] *= (*layer_)(lin_idx);
          //std::cout << "dir prob(" << i << ") = " << direction_probabilities_[i] << std::endl;
          
          it_steps++;
        }
        // normalize bc of line iterator steps
        // also dep. on velocity length might be different
        direction_probabilities_[i] /= static_cast<double>(it_steps); 
        direction_probabilities_[i] *= weight_factor_[i];
        
        if(direction_probabilities_.back() != 0.0)
        {
          all_zero = false;
        }
      }
      /*
      if(heat_map_.getIndex(next_pos, next_idx))
      {        
        const size_t lin_idx = grid_map::getLinearIndexFromIndex(next_idx, heat_map_.getSize());
        
        direction_probabilities_.at(i) = (*layer_)(lin_idx) * weight_factor_.at(i);
        
        if(direction_probabilities_.back() != 0.0)
        {
          all_zero = false;
        }
        
      }*/
      else
      {
        direction_probabilities_[i] = 0.0;
      }
    }

    if (!all_zero)
    {
      std::discrete_distribution<int> direction_distribution(direction_probabilities_.begin(),
                                                             direction_probabilities_.end());
      direction = direction_distribution(generator_);
    }
    else
    {
      direction = 0;
    }
  }
  else
  {
    direction = 0;
  }

  double theta_diff = direction * 2 * M_PI / angle_partitions_;
  
  double theta_next = theta + atan2(sin(theta_diff), cos(theta_diff)) * gamma_;
  
  sample(static_cast<int>(State::X)) = state(static_cast<int>(State::X)) + state(static_cast<int>(State::VX)) * dt + dt * dt * 0.5 * sigma_x_ * sigma_x_ * normal_distribution_(generator_);
  sample(static_cast<int>(State::Y)) = state(static_cast<int>(State::Y)) + state(static_cast<int>(State::VY)) * dt + dt * dt * 0.5 * sigma_y_ * sigma_y_ * normal_distribution_(generator_);

  sample(static_cast<int>(State::VX)) = r * cos(theta_next) + dt * sigma_x_ * sigma_x_ * normal_distribution_(generator_);
  sample(static_cast<int>(State::VY)) = r * sin(theta_next) + dt * sigma_y_ * sigma_y_ * normal_distribution_(generator_);

  state = sample;
}
