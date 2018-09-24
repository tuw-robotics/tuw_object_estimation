/*
 * Original Solution from stackoverflow:
 * https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
 * requires c++14
 */

#include <eigen3/Eigen/Dense>
#include <random>

/*!
 * @brief Generates multivariate Gaussian Samples
 */
template<typename T> struct MultivariateGaussianSampler {
    MultivariateGaussianSampler ( Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> const& covar )
        : MultivariateGaussianSampler ( Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero ( covar.rows() ), covar )
    {}

    MultivariateGaussianSampler ( Eigen::Matrix<T, Eigen::Dynamic, 1> const& mean, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> const& covar )
        : mean ( mean )
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> eigenSolver ( covar );
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::Matrix<T, Eigen::Dynamic, 1> mean;
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> transform;

    Eigen::Matrix<T, Eigen::Dynamic, 1> sample() const
    {
        std::mt19937 gen { std::random_device{}() };
        std::normal_distribution<T> dist;

        return mean + transform * Eigen::Matrix<T, Eigen::Dynamic, 1> { mean.size() } .unaryExpr ( [&] ( auto x ) {
            return dist ( gen );
        } );
    }
};
