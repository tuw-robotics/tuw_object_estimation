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

#include "heatmap_system_model_inv.h"
#include <boost/math/distributions/normal.hpp>

HeatMapSystemModelInv::HeatMapSystemModelInv(double sigma_theta, int angle_partitions, grid_map::GridMap& heat_map, grid_map::Matrix* layer)
{
  std::random_device rd;
  generator_ = std::mt19937(rd());
  normal_distribution_ = std::normal_distribution<double>();
  sigma_theta_ = sigma_theta;
  heat_map_ = heat_map;
  layer_ = layer;
  angle_partitions_ = angle_partitions;
  weight_factor_.resize(angle_partitions);
  
  sum_for_weight_factor_ = 0;
  
  // precompute weight factors
  for(int i = 0; i < angle_partitions_; i++)
  {    
    double angle = atan2(sin(i * 2 * M_PI / angle_partitions_), cos(i * 2 * M_PI / angle_partitions_));
    
    boost::math::normal norm = boost::math::normal(0, sigma_theta_ * sigma_theta_);
    weight_factor_.at(i) = boost::math::pdf(norm, angle);
  }
}


double HeatMapSystemModelInv::getProbability(const Eigen::Ref<const Eigen::VectorXd>& curr_state, const Eigen::Ref<const Eigen::VectorXd>& next_state, double dt)
{
  // current state vel vector angle theta and magnitude r
  double theta = atan2(curr_state(static_cast<int>(State::VY)), curr_state(static_cast<int>(State::VX)));
  double theta_tar = atan2(next_state(static_cast<int>(State::VY)), curr_state(static_cast<int>(State::VX)));
  
  // calculate map index from current position
  grid_map::Index idx;
  grid_map::Index next_idx;
  grid_map::Position pos(curr_state(static_cast<int>(State::X)), curr_state(static_cast<int>(State::Y)));
  grid_map::Position next_pos(next_state(static_cast<int>(State::X)), next_state(static_cast<int>(State::Y)));
  
  double theta_diff = theta_tar - theta;
  theta_diff = atan2(sin(theta_diff), cos(theta_diff));
  
  //double r = sqrt(curr_state(static_cast<int>(State::VX)) * curr_state(static_cast<int>(State::VX)) + curr_state(static_cast<int>(State::VY)) * curr_state(static_cast<int>(State::VY)));
  
  
  // TODO: check if new position is even reachable in dt (or 2 * dt)
  
  // check in which interval theta_diff is
  int i = 0;
  while(true)
  {
    if(theta_diff < i * 2 * M_PI / angle_partitions_)
    {
      i++;
    }
    else
    {
      break;
    }
  }
   
  probability_ = 1.0;

  if (heat_map_.getIndex(pos, idx))
  {
    if(heat_map_.getIndex(next_pos, next_idx))
    {
      unsigned int it_steps = 0;
      for(grid_map::LineIterator it(heat_map_, idx, next_idx); !it.isPastEnd(); ++it)
      {
        const grid_map::Index current_idx(*it);
        const size_t lin_idx = grid_map::getLinearIndexFromIndex(current_idx, heat_map_.getSize());
        
        probability_ *= (*layer_)(lin_idx);
        //std::cout << "dir prob(" << i << ") = " << direction_probabilities_[i] << std::endl;
        
        it_steps++;
      }
      // normalize bc of line iterator steps
      // also dep. on velocity length might be different
      probability_ /= static_cast<double>(it_steps); 
      probability_ *= weight_factor_[i];
    }
  }
  else
  {
    probability_ = 0.0;
  }
  
  return probability_;
}

