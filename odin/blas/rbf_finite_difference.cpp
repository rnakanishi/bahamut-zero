#include <blas/rbf_finite_difference.hpp>
#include <iostream>

namespace Odin {
DifferentialRBF2::DifferentialRBF2() {
  _isScalarField = _isVectorField = false;
  _kernelType = DifferentialRBF2::Kernel::QUINTIC;
  _polynomialType = Polynomial::QUADRATIC;
}

DifferentialRBF2::~DifferentialRBF2() {}

void DifferentialRBF2::setKernel(DifferentialRBF2::Kernel newKernel) {
  _kernelType = newKernel;
}

void DifferentialRBF2::addSamplePoint(std::vector<Eigen::Array2d> samplePoints,
                                      std::vector<double> scalarValues) {
  try {
    for (int pId = 0; pId < samplePoints.size(); pId++) {
      addSamplePoint(samplePoints[pId], scalarValues[pId]);
    }
  } catch (Bahamut::BahamutException& exception) {
    exception.setCallingLocation(
        "DifferentialRBF2::addSamplePoint(std::vector<Eigen::Array2d>, "
        "std::vector<double>)");
    throw(exception);
  }
}

void DifferentialRBF2::addSamplePoint(Eigen::Array2d samplePoint,
                                      double scalarValue) {
  if (!_isScalarField && !_isVectorField)
    _isScalarField = true;
  if (_isVectorField) {
    Bahamut::ConditionsNotMatchException exception(
        "Different type value already set!",
        "DifferentialRBF2::addSamplePoint(Eigen::Array2d, double)");
    exception.setErrorNumber(401);
    exception.setLethality(true);
    throw(exception);
  }
  _samplePoints.emplace_back(samplePoint);
  _sampleScalarValues.emplace_back(scalarValue);
}

void DifferentialRBF2::addSamplePoint(
    std::vector<Eigen::Array2d> samplePoints,
    std::vector<Eigen::Vector2d> scalarValues) {
  try {
    for (int pId = 0; pId < samplePoints.size(); pId++) {
      addSamplePoint(samplePoints[pId], scalarValues[pId]);
    }
  } catch (Bahamut::BahamutException& exception) {
    exception.setCallingLocation(
        "DifferentialRBF2::addSamplePoint(std::vector<Eigen::Array2d>, "
        "std::vector<Eigen::Vector2d>)");
    throw(exception);
  }
}

void DifferentialRBF2::addSamplePoint(Eigen::Array2d samplePoint,
                                      Eigen::Vector2d scalarValue) {
  if (!_isScalarField && !_isVectorField)
    _isVectorField = true;
  if (_isScalarField) {
    Bahamut::ConditionsNotMatchException exception(
        401,
        "DifferentialRBF2::addSamplePoint(Eigen::Array2d, Eigen::Vector2d)");
    exception.setLethality(true);
    throw(exception);
  }
  _samplePoints.emplace_back(samplePoint);
  _sampleVectorValues.emplace_back(scalarValue);
}

void DifferentialRBF2::clearSamples() {
  _isScalarField = false;
  _isVectorField = false;
  _samplePoints.clear();
  _sampleScalarValues.clear();
  _sampleVectorValues.clear();
}

int DifferentialRBF2::getSamplePointSize() {
  return _samplePoints.size();
}

size_t DifferentialRBF2::getPolynomialBaseSize() {
  if (_polynomialType == Polynomial::CONSTANT)
    return 1;
  if (_polynomialType == Polynomial::LINEAR)
    return 3;
  if (_polynomialType == Polynomial::QUADRATIC)
    return 6;
  return 1;
}

Eigen::Array2d DifferentialRBF2::computeGradientAt(Eigen::Array2d target) {
  size_t nPoints = _samplePoints.size();
  if (nPoints < 3) {
    throw(Bahamut::ConditionsNotMatchException(
        402, "DifferentialRBF2::computeGradientAt"));
  }
  // Normalization to put the target at the origin. This generates better
  // results for the interpolation
  std::vector<Eigen::Array2d> normalizedPoint;
  for (auto& point : _samplePoints) {
    normalizedPoint.push_back(point - target);
  }
  target = Eigen::Array2d(0., 0.);

  // Add polynomial function to the matrix
  size_t polySize = getPolynomialBaseSize();

  // Build the distance matrix
  Eigen::MatrixXd M(nPoints + polySize, nPoints + polySize);

  // Solve the system based on the sample values
}

std::vector<double> DifferentialRBF2::computeLaplacianWeightsAt(
    Eigen::Array2d target) {
  int nPoints = _samplePoints.size();
  if (nPoints < 3) {
    throw(Bahamut::ConditionsNotMatchException(
        402, "DifferentialRBF2::computeLaplacianWeights"));
  }
  // Target is translated in origin position
  // sample points are centered over the target (0,0)
  std::vector<Eigen::Vector2d> points;
  for (auto& point : _samplePoints) {
    points.push_back(point.matrix() - target.matrix());
  }

  // Set polynomial base and get respective polynomials
  setPolynomialBase(DifferentialRBF2::Polynomial::QUADRATIC);
  int baseSize = getPolynomialBaseSize();

  Eigen::MatrixXd M(nPoints + baseSize, nPoints + baseSize);
  Eigen::VectorXd b(nPoints + baseSize);
  M.setZero();
  b.setZero();

  // Assemble the system
  // M(i,j) = phi(| xi - xj |)
  // b = d2phi + (1/|xi-xj|) dphi
  for (size_t i = 0; i < points.size(); i++) {
    b[i] = computeKernelSecondDerivative(points[i].norm());
    if (!points[i].isApproxToConstant(0., 1e-8)) {
      b[i] += computeKernelFirstDerivative(points[i].norm()) / points[i].norm();
    }
    for (size_t j = 0; j < points.size(); j++) {
      M(i, j) = M(j, i) = computeKernel((points[i] - points[j]).norm());
    }

    // Add polynomial augmentation to the matrix and the laplacian counter part
    // to the b vector
    std::vector<double> polynomial = computePolynomial(points[i]);
    std::vector<double> dxx =
        computeSecondDerivative(DifferentialRBF2::Derivative::DXX, points[i]);
    std::vector<double> dyy =
        computeSecondDerivative(DifferentialRBF2::Derivative::DYY, points[i]);
    for (size_t p = 0; p < baseSize; p++) {
      M(nPoints + p, i) = M(i, nPoints + p) = polynomial[p];
      b[nPoints + p] = dxx[p] + dyy[p];
    }
  }

  // Solving the system
  // LU is used because precision is required
  // Householder QR is another option. But for some kernels, it may return
  // imprecise results
  Eigen::VectorXd weights = M.fullPivLu().solve(b);
  if (weights.hasNaN()) {
    throw(Bahamut::BadResultException(
        501, "DifferentialRBF2::computeLaplacianWeights"));
  }
  std::vector<double> finalWeights(weights.data(),
                                   weights.data() + weights.size() - baseSize);
  return finalWeights;
}

}  // namespace Odin
