#include "../../../include/aon/math/scaling/exponential-scaler.hpp"

namespace aon {

double ExponentialScaler::transform(double value) {
  value = ::std::clamp(value, -127.0, 127.0); // Normalizing input
  const double sign = (value > 0) - (value < 0);
  const double result = ::std::exp((::std::fabs(value) - 127.0) * t / 1000.0);

  return sign * ::std::clamp(result, -1.0, 1.0);
}

} // namespace aon