#ifndef __ODIN_INTERPOLATION_INTERPOLATION_INTERFACE_HPP__
#define __ODIN_INTERPOLATION_INTERPOLATION_INTERFACE_HPP__

#include <Eigen/Dense>
#include <vector>

namespace Odin {
class InterpolationInterface2 {
 public:
  /**
   * @brief Interpolation function interface. For a given target in 2D space,
   * return its value interpolated using the vector of points and their
   * respective function values that should be provided. If the target is
   * extremely close to some of  the points, then its value is returned instead
   * of computing it.
   *
   * @param target point in space where the interpolation is desired
   * @param samples vector containing the location of the points that will be
   * used for the interpolation method
   * @param values respective function values at the provided samples
   * @return double the result of the interpolation method
   */
  virtual double interpolateAt(Eigen::Array2d target,
                               std::vector<Eigen::Array2d> samples,
                               std::vector<double> values) = 0;

 protected:
};

// ===========================================================================
// ===========================================================================

class InterpolationInterface3 {
 public:
  InterpolationInterface3();

  virtual ~InterpolationInterface3();

  /**
   * @brief Interpolation function interface. For a given target in 3D space,
   * return its value interpolated using the vector of points and their
   * respective function values that should be provided. If the target is
   * extremely close to some of  the points, then its value is returned instead
   * of computing it.
   *
   * @param target point in space where the interpolation is desired
   * @param samples vector containing the location of the points that will be
   * used for the interpolation method
   * @param values respective function values at the provided samples
   * @return double the result of the interpolation method
   */
  virtual double interpolateAt(Eigen::Array3d target,
                               std::vector<Eigen::Array3d> samples,
                               std::vector<double> values) = 0;

 protected:
};

}  // namespace Odin

#endif