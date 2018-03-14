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

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Dynamic;
using Eigen::RowMajor;
using Eigen::Ref;

template <int dim>
class MeanShift
{

public:
  MeanShift(int num_datapoints, double kernel_bandwidth)
  {
    num_datapoints_ = num_datapoints;
    kernel_bandwidth_ = kernel_bandwidth;
  }
  
  /*
   * @brief Calculates neighborhood for all data points and shifts them according to MeanShift algorithm.
   * @param data Input matrix holding all data points to be clustered. Also used as output
   *        containing the shifted points, hence should be a copy of original data.
   * @param iterations Number of shift iterations
   * @param thresh Neighborhood distance threshold
   */
  void meanShift(Ref<Matrix<double, dim, Dynamic, RowMajor>> data, int iterations, double thresh) const
  {
    for(int i = 0; i < iterations; i++)
    {
      for(int feature_idx = 0; feature_idx < num_datapoints_; feature_idx++)
      {
        Matrix<double, dim, 1> feature = data.col(feature_idx);
        Matrix<double, 1, Dynamic, RowMajor> N_dist;
        Matrix<double, dim, Dynamic, RowMajor> N;
        
        // calculate neighbors of feature
        euclideanNeighborhood(feature, data, N_dist, N, thresh);
        
        Matrix<double, dim, 1> nom = Matrix<double, dim, 1>::Zero();
        double denom = 0;
        
        N_dist.unaryExpr(std::bind(&MeanShift::gaussianKernel, this, std::placeholders::_1));
        denom = N_dist.sum();
        nom = N * N_dist.transpose();
        
        data.col(feature_idx) = nom / denom;
      }
    }
  }
  
  /*
   * @brief Extracts cluster centers from shifted data points.
   * @param data Previously shifted datapoints by meanShift.
   * @param cluster_centers Output Vector of cluster centers.
   * @param thresh Threshold for distinguishing center points
   */
  void extractClusterCenters(Ref<Matrix<double, dim, Dynamic, RowMajor>> data, std::vector<Matrix<double, dim, 1>>& cluster_centers, double thresh) const
  {
    for(int feature_idx = 0; feature_idx < num_datapoints_; feature_idx++)
    {
      size_t i = 0;
      for(; i < cluster_centers.size(); i++)
      {
        if((data.col(feature_idx) - cluster_centers[i]).norm() < thresh)
        {
          break;
        }
      }

      if(i == cluster_centers.size())
      {
        cluster_centers.emplace_back(data.col(feature_idx));
      }
    }
  }
  
private:  
  
  int num_datapoints_;
  double kernel_bandwidth_;
  
  double gaussianKernel(double distance) const
  {
    return std::exp(-1.0 / 2.0 * (distance * distance) / (kernel_bandwidth_ * kernel_bandwidth_));
  }
  
  /*
   * @brief Calculates euclidean distances for a point. Discard points further away than thresh
   * 
   * @param p Point for which the neighborhood should be calculated.
   * @param candidates Neighborhood candidate points.
   * @param N_dist Output matrix holding the distances for all points in the neighborhood of p.
   * @param N Ouput matrix holding points in the neighborhood of p.
   * @param thresh Distance threshold for points to be considered in the neighborhood of p.
   */
  void euclideanNeighborhood(Ref<Matrix<double, dim, 1>> p, Ref<Matrix<double, dim, Dynamic, RowMajor>> candidates, Matrix<double, 1, Dynamic, RowMajor>& N_dist, Matrix<double, dim, Dynamic, RowMajor>& N, double thresh) const
  {
    Matrix<bool, 1, Dynamic, RowMajor> neighbors_mask;
    //neighbors_mask.resize(1, num_datapoints_);
    
    Matrix<double, 1, Dynamic, RowMajor> distances = (candidates.colwise() - p).colwise().norm();
    
    neighbors_mask = (distances.array() < thresh).matrix();
    
    const int num_neighbors = (distances.array() < thresh).count();
    Matrix<double, dim, Dynamic, RowMajor> neighbors;
    neighbors.resize(dim, num_neighbors);
    Matrix<double, 1, Dynamic, RowMajor> neighbors_dist;
    neighbors_dist.resize(1, num_neighbors);
    
    int candidates_idx = 0;
    for(int i = 0; i < num_neighbors; i++)
    {
      while(!neighbors_mask(candidates_idx) && candidates_idx < num_datapoints_)
        candidates_idx++;
      
      neighbors_dist.col(i) = distances.col(candidates_idx);
      neighbors.col(i) = candidates.col(candidates_idx);
      candidates_idx++;
    }
    
    N_dist = neighbors_dist;
    N = neighbors;
  }
};
