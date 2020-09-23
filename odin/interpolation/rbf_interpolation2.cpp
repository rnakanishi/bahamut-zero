#include <interpolation/rbf_interpolation.hpp>
#include <exceptions/exception.hpp>

namespace Odin {
RBFInterpolation2::RBFInterpolation2() {
  _kernel = Kernel::QUINTIC;
  _polynomialDegree = 1;
}
RBFInterpolation2::~RBFInterpolation2() {}

double RBFInterpolation2::interpolateAt(Eigen::Array2d target,
                                        std::vector<Eigen::Array2d> samples,
                                        std::vector<double>) {
  size_t nPoints = samples.size();

  // Check which kernel is being used here

  // Check which polynomial base is being used here

  return 0.0;
}

void RBFInterpolation2::setKernel(RBFInterpolation2::Kernel newKernel) {
  _kernel = newKernel;
}

void RBFInterpolation2::setPolynomialDegreee(int newDegree) {
  if (newDegree < 0 || newDegree > 2) {
    _polynomialDegree = 1;
    Bahamut::UnexpectedParameterException exception(
        "Polynomial degree not supported",
        "RBFInterpolation2::setPolynomialDegree");
    exception.setErrorNumber(301);
    exception.setLethality(false);
    throw(exception);
  }
  _polynomialDegree = newDegree;
}

void RBFInterpolation2::evaluateKernel() {}

}  // namespace Odin
