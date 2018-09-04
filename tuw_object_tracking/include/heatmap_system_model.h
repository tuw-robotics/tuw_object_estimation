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

#ifndef HEATMAP_SYSTEM_MODEL_H
#define HEATMAP_SYSTEM_MODEL_H

#include <system_model.h>
#include <grid_map_ros/grid_map_ros.hpp>

class HeatMapSystemModel : public SystemModel
{
public:
  HeatMapSystemModel(double sigma_x, double sigma_y, double sigma_theta, int angle_partitions, double gamma, grid_map::GridMap& heat_map, grid_map::Matrix* layer);

  void sample(Eigen::Ref<Eigen::VectorXd> state, double dt, const Eigen::Ref<const Eigen::VectorXd>& meas, const Eigen::Ref<const Eigen::MatrixXd>& meas_cov) override;

private:
  std::mt19937 generator_;                                /// random number generator
  std::normal_distribution<double> normal_distribution_;  /// normal distribution for generic use
  double sigma_x_;
  double sigma_y_;
  double sigma_theta_;
  grid_map::GridMap heat_map_;
  grid_map::Matrix* layer_;
  int angle_partitions_;
  std::vector<double> direction_probabilities_;
  std::vector<double> weight_factor_;
  double sum_for_weight_factor_;
  double gamma_;
};

#endif  // HEATMAP_SYSTEM_MODEL_H
