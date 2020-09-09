#ifndef __ODIN_RBF_FINITE_DIFFERENCE_HPP__
#define __ODIN_RBF_FINITE_DIFFERENCE_HPP__

#include <Eigen/Dense>
#include <utils/exception.hpp>
#include <vector>

namespace Odin {
class DifferentialRBF2 {
 public:
  /**
   * @brief Kernel function to be used by the interpolation. The type of the
   * function may affect the result, since semi definite kernels may need more
   * symmetric samples. While definite positive kernels are granted to give
   * better results at the cost of speed
   *
   * TODO: Create a separate class for kernels and their methods
   */
  enum class Kernel { QUINTIC };

  /**
   * @brief Defines the polynomial base to be used during the interpolation
   * step. Note that the polynomial base may be affected by the number of
   * samples: lower polynomials need less samples, but gives lower accuracy
   * results. On the other hand, if high order polynomials are used with low
   * sample count, the results may be worse than expected
   *
   * Constant polynomial: 1
   * Linear polynomial: 1 x y
   * Quadratic polynomial: 1 x y x2 y2 xy
   *
   * TODO: Create a separate class for polynomials and their methods
   */
  enum class Polynomial { CONSTANT, LINEAR, QUADRATIC };

  enum class Derivative { DX, DY, DXX, DYY, DXY, DYX };

  DifferentialRBF2();

  ~DifferentialRBF2();

  /**
   * @brief Changes the kernel to be used in the RBF computations.
   * Currently supported kernels:
   *   - Quintic: polyharmonic function, $x^5$
   *
   * @param newKernel enum value of the desired kernel
   */
  void setKernel(Kernel newKernel);

  /**
   * @brief Set the Polynomial Base object for the RBF differential
   * computations. Different degrees of polynomials may require more or less
   * sample points. If high degree polynomials are used with lower sample count,
   * the result may be worse than using lower degree polynomials.
   *
   * @param newPolynomial polynomial type to be assigned
   */
  void setPolynomialBase(Polynomial newPolynomial);

  std::vector<double> computePolynomial(Eigen::Array2d point);

  std::vector<double> computeFirstDerivative(Derivative derivative,
                                             Eigen::Array2d point);

  std::vector<double> computeSecondDerivative(Derivative derivative,
                                              Eigen::Array2d point);

  double computeKernel(double distance);

  double computeKernelFirstDerivative(double distance);

  double computeKernelSecondDerivative(double distance);

  /**
   * @brief Adds a sample point and its respective value, be it a scalar value
   * or a vector value. The first type of function value sets the overall
   * instatiation type: if an attempt to add the other type of data is made, an
   * exception is thrown and the data is not set.
   *
   * Multiple datas can be added if a std::vector container is given as
   * parameter. The size of the sample_points container and the values container
   * must be the same, otherwise, an exception is thrown
   *f
   * @param sample_points: either a single point or a container with the points
   * @param function_value: can be either scalar type or vector type.
   * Disregarding the type, if a container is given for the sample points, the
   * number of values must match that size
   */
  void addSamplePoint(Eigen::Array2d samplePoint, double scalarValue);
  void addSamplePoint(std::vector<Eigen::Array2d> samplePoints,
                      std::vector<double> scalarValues);
  void addSamplePoint(Eigen::Array2d samplePoint, Eigen::Vector2d vectorValue);
  void addSamplePoint(std::vector<Eigen::Array2d> samplePoints,
                      std::vector<Eigen::Vector2d> vectorValues);

  /**
   * @brief Clear all sample points and their respective function values. The
   * data type is also reset on this method call.
   * Polynomial type and kernel are kept the same
   */
  void clearSamples();

  /**
   * @brief Counts how many sample points were provided to this instance until
   * the calling of this method.
   *
   * @return int total count of sample points
   */
  int getSamplePointSize();

  /**
   * @brief Get the Polynomial Base Size.
   * Constant polynomial: 1
   * Linear polynomial: 1 x y
   * Quadratic polynomial: 1 x y x2 y2 xy
   *
   * @return size_t size of the polynomial base for 2 dimensions
   */
  size_t getPolynomialBaseSize();

  /**
   * @brief Given that the data type is set to scalar, compute the gradient of
   * the function at the points evaluated at the target parameter position. If
   * vector data type is set instead, a Warning exception is thrown
   *
   * @param target point where the gradient information will be computed
   * @return Eigen::Array2d the gradient of the provided data
   */
  Eigen::Array2d computeGradientAt(Eigen::Array2d target);

  /**
   * @brief Given that th data type is scalar, compute the Laplacian weights to
   * be used in Poisson based equations at the points specified in target
   * parameter. The weights are returned in respective to the order of the given
   * samples.
   *
   * @param target point where the weights will be computed
   * @return std::vector<double> final weights of the given samples
   */
  std::vector<double> computeLaplacianWeightsAt(Eigen::Array2d target);

 protected:
  Kernel _kernelType;
  Polynomial _polynomialType;
  int _polynomialBase;
  bool _isScalarField;
  bool _isVectorField;
  std::vector<Eigen::Array2d> _samplePoints;
  std::vector<double> _sampleScalarValues;
  std::vector<Eigen::Vector2d> _sampleVectorValues;
};
}  // namespace Odin

#endif
