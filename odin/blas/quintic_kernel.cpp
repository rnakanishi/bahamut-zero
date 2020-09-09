#include <blas/rbf_finite_difference.hpp>

namespace Odin {

double DifferentialRBF2::computeKernel(double distance) {
  double d2 = distance * distance;
  return d2 * d2 * distance;
}

double DifferentialRBF2::computeKernelFirstDerivative(double distance) {
  double d2 = distance * distance;
  return 5 * d2 * d2;
}

double DifferentialRBF2::computeKernelSecondDerivative(double distance) {
  return 20 * distance * distance * distance;
}

}  // namespace Odin
