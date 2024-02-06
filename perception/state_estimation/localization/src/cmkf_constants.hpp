#pragma once

#include <Eigen/Dense>

namespace localization
{

#define NUM_DIM_ROBOT 3

#define ME_X_DIM 0
#define ME_Y_DIM 1
#define ME_H_DIM 2

#define FF_DISTANCE_DIM 0
#define FF_HEADING_DIM 1
#define FF_ORIENTATION_DIM 2
#define FF_DIM_TOTAL 3

using StateVector = Eigen::Matrix<float, NUM_DIM_ROBOT, 1>;
using CovarianceMatrix = Eigen::Matrix<float, NUM_DIM_ROBOT, NUM_DIM_ROBOT>;

}  // namespace localization
