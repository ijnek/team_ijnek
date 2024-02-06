#pragma once

#include "cmkf_constants.hpp"

namespace localization
{

class CMKF
{
public:
  CMKF(
    StateVector state,
    CovarianceMatrix covariance,
    float weight);

  StateVector state;
  CovarianceMatrix covariance;
  float weight;
};

}  // namespace localization
