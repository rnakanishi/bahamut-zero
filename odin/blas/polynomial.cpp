#include <blas/rbf_finite_difference.hpp>

namespace Giratina {

void DifferentialRBF2::setPolynomialBase(Polynomial newPolynomial) {
  _polynomialType = newPolynomial;
}

std::vector<double> DifferentialRBF2::computePolynomial(Eigen::Array2d point) {
  double x = point[0];
  double y = point[1];
  switch (_polynomialType) {
    case DifferentialRBF2::Polynomial::CONSTANT:
      return {1.};
    case DifferentialRBF2::Polynomial::LINEAR:
      return {1, x, y};
    case DifferentialRBF2::Polynomial::QUADRATIC:
      return {1, x, y, x * x, y * y, x * y};
  }
}

std::vector<double> DifferentialRBF2::computeFirstDerivative(
    DifferentialRBF2::Derivative derivative,
    Eigen::Array2d point) {
  if (derivative == DifferentialRBF2::Derivative::DXX ||
      derivative == DifferentialRBF2::Derivative::DXY ||
      derivative == DifferentialRBF2::Derivative::DYX ||
      derivative == DifferentialRBF2::Derivative::DYY) {
    // First derivative parameters are not supported
    Arceus::UnexpectedParameterException exception(
        303, "DifferentialRBF2::computeFirstDerivative");
    exception.setLethality(false);
    throw(exception);
  }

  switch (_polynomialType) {
    case DifferentialRBF2::Polynomial::CONSTANT:
      // Constant type is the same for all firs derivatives
      return {0.};
    case DifferentialRBF2::Polynomial::LINEAR:
      // Treatment needed for linear case
      switch (derivative) {
        case DifferentialRBF2::Derivative::DX:
          return {0., 1., 0.};
        case DifferentialRBF2::Derivative::DY:
          return {0., 0., 1.};
      }
      return {0., 0., 0.};
    case DifferentialRBF2::Polynomial::QUADRATIC:
      // Quadratic case treats all values as well and use point location info
      switch (derivative) {
        case DifferentialRBF2::Derivative::DX:
          return {0., 1., 0., 2. * point[0], 0., point[1]};
        case DifferentialRBF2::Derivative::DY:
          return {0., 0., 1., 0., 2. * point[1], point[0]};
      }
  }
}

std::vector<double> DifferentialRBF2::computeSecondDerivative(
    DifferentialRBF2::Derivative derivative,
    Eigen::Array2d point) {
  if (derivative == DifferentialRBF2::Derivative::DX ||
      derivative == DifferentialRBF2::Derivative::DY) {
    // First derivative parameters are not supported
    Arceus::UnexpectedParameterException exception(
        303, "DifferentialRBF2::computeSecondDerivative");
    exception.setLethality(false);
    throw(exception);
  }

  switch (_polynomialType) {
    case DifferentialRBF2::Polynomial::CONSTANT:
      // Constant type is the same for all second derivatives
      return {0.};
    case DifferentialRBF2::Polynomial::LINEAR:
      // The same for the linear case
      return {0., 0., 0.};
    case DifferentialRBF2::Polynomial::QUADRATIC:
      // Quadratic case needs treatment for each derivative value
      switch (derivative) {
        case DifferentialRBF2::Derivative::DXX:
          return {0., 0., 0., 2., 0., 0.};
        case DifferentialRBF2::Derivative::DYY:
          return {0., 0., 0., 0., 2., 0.};
        case DifferentialRBF2::Derivative::DXY:
        case DifferentialRBF2::Derivative::DYX:
          return {0., 0., 0., 0., 0., 1.};
      }
  }
}
}  // namespace Giratina