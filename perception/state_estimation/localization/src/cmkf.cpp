#include "cmkf.hpp"

namespace localization
{

CMKF::CMKF(StateVector state, CovarianceMatrix covariance, float weight)
  : state(state), covariance(covariance), weight(weight)
{
}

}  // namespace localization
