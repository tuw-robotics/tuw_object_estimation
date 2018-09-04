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

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <particle_filter_config.h>
#include <tuw_geometry/tuw_geometry.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include "common.h"
#include "mean_shift.h"

using namespace tuw;

template <int stateDim>
class ParticleFilter
{
public:
  struct Particle
  {
    Eigen::Matrix<double, stateDim, 1> state;
    double weight;
  };

  ParticleFilter(std::shared_ptr<ParticleFilterConfig> config)
    : system_model_(config->system_model)
    , meas_model_(config->meas_model)
    , system_model_inv_(config->system_model_inv)
    , meas_model_inv_(config->meas_model_inv)
  {
    std::random_device rd;
    generator_ = std::mt19937(rd());
    normal_distribution_ = std::normal_distribution<double>();

    num_particles_ = config->num_particles;
    resample_rate_ = config->resample_rate;
    particles_ = std::make_shared<std::vector<Particle>>(num_particles_);
    cov_ = std::make_shared<Eigen::Matrix<double, stateDim, stateDim>>();
    mean_ = std::make_shared<Eigen::Matrix<double, stateDim, 1>>();
    cluster_centroids_ = std::make_shared<std::vector<Eigen::Matrix<double, 2, 1>>>();
    const_fwd_pred_ = config->const_fwd_pred;
    fwd_pred_time_ = config->fwd_pred_time;
    particle_filter_output_modality_ = config->particle_filter_output_modality;
    enable_clustering_config_ = config->enable_clustering;
    enable_clustering_ = false;

    weight_sum_ = 1.0;
  }

  void updateConfig(std::shared_ptr<ParticleFilterConfig> config)
  {
    system_model_ = config->system_model;
    meas_model_ = config->meas_model;
    num_particles_ = config->num_particles;
    resample_rate_ = config->resample_rate;
    particles_->resize(config->num_particles);
    const_fwd_pred_ = config->const_fwd_pred;
    fwd_pred_time_ = config->fwd_pred_time;
    particle_filter_output_modality_ = config->particle_filter_output_modality;
  }

  // generate normal distributed particles around init_state
  void initAroundState(const Eigen::Matrix<double, stateDim, 1>& init_state, double sigma)
  {
    Eigen::Matrix<double, stateDim, 1> temp_state;
    for (int i = 0; i < num_particles_; i++)
    {
      for (int j = 0; j < init_state.size(); j++)
      {
        temp_state(j) = init_state(j) + normal_distribution_(generator_) * sigma;
      }
      (*particles_)[i].state = temp_state;
      (*particles_)[i].weight = 1.0 / num_particles_;
    }
    updateMeanAndCov();
  }

  void predict(boost::posix_time::ptime current_time_stamp)
  {
    (void)updateTimestamp(current_time_stamp);

    double dt;

    if (const_fwd_pred_)
    {
      dt = boost::posix_time::millisec(fwd_pred_time_ * 1000).total_microseconds() / 1000000.;
    }
    else
    {
      dt = (duration_last_update_ + boost::posix_time::millisec(fwd_pred_time_ * 1000)).total_microseconds() / 1000000.;
    }

    if (dt <= 0.0)
      return;

    resample();

    // forward prediction
    for (size_t i = 0; i < particles_->size(); i++)
    {
      system_model_->sample((*particles_)[i].state, dt);
    }
  }

  void update(const MeasurementObject::Object& meas, bool dummy_meas)
  {
    Eigen::Vector2d meas_vec;

    if (!dummy_meas)
    {
      weight_sum_ = 0.0;

      meas_vec(0) = meas.pose2d.x();
      meas_vec(1) = meas.pose2d.y();
    }

    // go through all samples, get importance factor (weight) through meas model
    for (size_t i = 0; i < particles_->size(); i++)
    {
      if (!dummy_meas)
      {
        (*particles_)[i].weight = meas_model_->getProbability((*particles_)[i].state, meas_vec, meas.covariance);
        weight_sum_ += (*particles_)[i].weight;
      }
      else
      {
        // weight_sum_ = 1.0;
        //(*particles_)[i].weight = 1.0 / num_particles_;
      }
    }

    if (!dummy_meas)
    {
      // normalize
      for (auto pit = particles_->begin(); pit != particles_->end(); pit++)
      {
        if (weight_sum_ != 0.0)
        {
          pit->weight /= weight_sum_;
        }
      }

      // sort particles_[0] has largest weight
      std::sort(particles_->begin(), particles_->end(), [&](Particle a, Particle b) { return (a.weight > b.weight); });
    }

    // output particle array
    /*
    for(auto&& it = particles_->begin(); it != particles_->end(); it++)
    {
      std::cout << "weight = " << it->weight << ", ";
    }
    std::cout << std::endl << "-------------------------------------------------" << std::endl;
    */

    // cluster if no meas, i.e. person possibly occluded
    if (dummy_meas && enable_clustering_ && enable_clustering_config_)
    {
      auto start = std::chrono::high_resolution_clock::now();
      cluster_centroids_ = std::make_shared<std::vector<Eigen::Matrix<double, 2, 1>>>();
      Eigen::Matrix<double, 2, Dynamic, Eigen::RowMajor> data;
      data.resize(2, num_particles_);
      for (size_t i = 0; i < particles_->size(); i++)
      {
        data.col(i) = (*particles_)[i].state.head(2);
      }

      MeanShift<2> mean_shift(num_particles_, 4.0);
      mean_shift.meanShift(data, 5, 1.5);
      mean_shift.extractClusterCenters(data, *cluster_centroids_, 0.5);

      if (cluster_centroids_->size() > 1)
      {
        std::cout << "found " << cluster_centroids_->size() << " clusters" << std::endl;

        for (size_t k = 0; k < cluster_centroids_->size(); k++)
        {
          std::cout << "cluster[" << k << "] = " << (*cluster_centroids_)[k] << std::endl;
        }
      }
      auto finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = finish - start;
      std::cout << "Elapsed time: " << elapsed.count() << " s" << std::endl;
    }

    updateMeanAndCov();
  }

  const Eigen::Matrix<double, stateDim, 1>& estimatedState() const
  {
    switch (particle_filter_output_modality_)
    {
      case 0:  // highest weight particle
        return (*particles_)[0].state;
      case 1:  // weighted mean over all particles
        return *mean_;
      case 2:  // weighted median over all particles
        return (*particles_)[particles_->size() / 2].state;
      default:
        std::cout << "wrong output modality, assuming mean" << std::endl;
        return *mean_;
    }
  }

  const std::shared_ptr<Eigen::Matrix<double, stateDim, stateDim>>& stateCovariance() const
  {
    return cov_;
  }

  const std::shared_ptr<Eigen::Matrix<double, stateDim, 1>>& stateMean() const
  {
    return mean_;
  }

  const std::shared_ptr<std::vector<Particle>>& particles() const
  {
    return particles_;
  }

  const boost::posix_time::ptime& lastUpdated() const
  {
    return timestamp_last_update_;
  }

  void setEnableClustering(bool enable_clustering)
  {
    enable_clustering_ = enable_clustering;
  }

private:
  void resample()
  {
    std::vector<Particle> resampled_particles;
    double M = resample_rate_ * particles_->size();
    std::uniform_real_distribution<double> r_dist(0, 1.0 / M);

    double r = r_dist(generator_);
    double c = (*particles_)[0].weight;
    double U;
    int i = 0;

    if (c > 0)
    {
      for (int m = 0; m < M; m++)
      {
        U = r + m * 1.0 / M;
        while (U > c)
        {
          i = (i + 1) % (*particles_).size();
          c = c + (*particles_)[i].weight;
        }
        // add resampled particle
        resampled_particles.push_back((*particles_)[i]);  // maybe add additional noise?
      }

      // replace particles
      for (int i = 0; i < M; i++)
      {
        (*particles_)[particles_->size() - i - 1] = resampled_particles[i];
      }
    }
  }

  void updateMeanAndCov()
  {
    double weight_square_sum = 0;

    // initialize mean / cov
    for (Eigen::Index i = 0; i < mean_->size(); i++)
    {
      for (Eigen::Index j = 0; j < mean_->size(); j++)
      {
        (*cov_)(i, j) = 0;
      }
      (*mean_)(i) = 0;
    }

    for (auto pit = particles_->begin(); pit != particles_->end(); pit++)
    {
      // std::cout << "weight: " << pit->weight << std::endl;
      // std::cout << "state: " << pit->state << std::endl;
      *mean_ += pit->state * pit->weight;
      weight_square_sum += pit->weight * pit->weight;
    }

    // calculate covariance
    for (auto pit = particles_->begin(); pit != particles_->end(); pit++)
    {
      *cov_ += pit->weight * (pit->state - *mean_) * (pit->state - *mean_).transpose();
    }

    if (weight_square_sum < 1.0)
    {
      *cov_ /= (1 - weight_square_sum);
    }
    else
    {
      // std::cout << "weight square sum = " << weight_square_sum << std::endl;
    }
  }

  /**
   * updates the timestamp_last_update_ and the duration_last_update_
   * on the first call it will set the timestamp_last_update_ to t and the duration_last_update_ to zero
   * the duration_last_update_ will be used for the prediction step
   * @return true on sucessful update, false on first use and if t is in the past
   **/
  bool updateTimestamp(const boost::posix_time::ptime& t)
  {
    if (timestamp_last_update_.is_not_a_date_time())
    {
      timestamp_last_update_ = t;
      // std::cout << "not a date time: " << std::endl;
    }
    if (timestamp_last_update_ < t)
    {
      duration_last_update_ = t - timestamp_last_update_;
      timestamp_last_update_ = t;
      return true;
    }
    else
    {
      return false;
    }
  }

  std::mt19937 generator_;                                /// random number generator
  std::normal_distribution<double> normal_distribution_;  /// normal distribution for generic use

  std::shared_ptr<SystemModel> system_model_;
  std::shared_ptr<MeasModel> meas_model_;
  std::shared_ptr<MeasModel> system_model_inv_;
  std::shared_ptr<SystemModel> meas_model_inv_;

  std::shared_ptr<std::vector<Particle>> particles_;
  std::shared_ptr<Eigen::Matrix<double, stateDim, stateDim>> cov_;
  std::shared_ptr<Eigen::Matrix<double, stateDim, 1>> mean_;

public:
  std::shared_ptr<std::vector<Eigen::Matrix<double, 2, 1>>> cluster_centroids_;

  double weight_sum_;
  int num_particles_;
  double resample_rate_;
  bool const_fwd_pred_;
  double fwd_pred_time_;
  int particle_filter_output_modality_;
  bool enable_clustering_config_;
  bool enable_clustering_;

  boost::posix_time::ptime timestamp_last_update_;         /// time of the last processed measurment
  boost::posix_time::time_duration duration_last_update_;  /// time since the previous processed measurment
};

#endif  // PARTICLE_FILTER_H
