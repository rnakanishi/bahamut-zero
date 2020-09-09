#ifndef __ODIN_RBF_INTERPOLATION_HPP__
#define __ODIN_RBF_INTERPOLATION_HPP__

#include <interpolation/interpolation_interface.hpp>

namespace Odin {

/**
 * @brief RBF interpolation class for 2D defined points
 *
 */
class RBFInterpolation2 : public InterpolationInterface2 {
 public:
  /**
   * @brief List of supported kernels in this class
   *
   */
  enum class Kernel { QUINTIC };

  /**
   * @brief Construct a new RBFInterpolation2 object. A defauld QUINTIC kernel
   * is set to this interpolator. If a different kernel is wanted, then it
   * should be changed before calling the interpolation function
   *
   */
  RBFInterpolation2();

  ~RBFInterpolation2();

  /**
   * @brief For a given target in 2D space, this method computes the distance
   * between all the points provided (target and samples) to build a distance
   * matrix. This matrix is used to solve the RBF system for the function values
   * given. As a result, the interpolated value at the target is obtained and
   * the returned.
   *
   * References:
   *   - Gregory F. Fasshauer. 2007. Meshfree Approximation Methods with
   * MATLAB. World Scientific Publishing Co., Inc., USA.
   *
   * @param target point in space where the interpolation is desired
   * @param samples vector containing the location of the points that will be
   * used for the interpolation method
   * @param values respective function values at the provided samples
   * @return double the result of the interpolation method
   */
  double interpolateAt(Eigen::Array2d target,
                       std::vector<Eigen::Array2d> samples,
                       std::vector<double>);

  /**
   * @brief Change the current kernel to a new one. This method should be called
   * before caling interpolateAt() method, otherwise, the default (or previous)
   * kernel will be used and unexpected results may occur.
   *
   * @param newKernel a supported kernel value (given as a enumerate of this
   * class). Non-supported kernels may thrown an exception
   */
  void setKernel(Kernel newKernel);

  /**
   * @brief Set the Polynomial Degreee to a new value (0, 1 or 2). This method
   * should be called before calling interpolateAt() method. Otherwise the
   * default value (or the previous one) will be used instead and unexpected
   * results may occur.
   *
   * @param newDegree 0, 1 or 2 for the respectives polynomial degrees. Other
   * values may throw and exception
   */
  void setPolynomialDegreee(int newDegree);

  void evaluateKernel();

 private:
  Kernel _kernel;
  int _polynomialDegree;
};

class RBFInterpolation3 : public InterpolationInterface3 {
 public:
  RBFInterpolation3();

 private:
};

}  // namespace Odin

#endif