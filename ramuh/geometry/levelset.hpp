#ifndef __PALKIA_LEVELSET_HPP__
#define __PALKIA_LEVELSET_HPP__

#include <Eigen/Dense>
#include <functional>
#include <geometry/bounding_box.hpp>
#include <grid/cell_centered_grid.hpp>

namespace Palkia {
class Levelset2 : public CellCenteredGrid2 {
 public:
  enum class Shape { CIRCLE, SQUARE };
  enum class Direction { HORIZONTAL, VERTICAL };

  Levelset2();
  Levelset2(BoundingBox2 domain, Eigen::Array2i gridSize);

  /**
   * @brief Given a cell Id that is on the levelset, checks if the cell is a
   * surface or not. If the cell's levelset is positive, a false value is
   * immediately returned. If the cell contains a negative levelset value, then
   * the method checks if any of the cell's neighbors contains a positive value.
   * If so, true is returnd.
   *
   * @param cellId query cell
   * @return true if the cell is a surface cell
   * @return false otherwise,
   */
  bool isSurface(size_t cellId);

  /**
   * @brief For all cells in the grid, set their valus based on the function
   * given as parameter
   * If a Shape class is given, then this method uses built-in functions. Care
   * for this case, because not all shapes may be implemented
   *
   * @param phi fucntion that returns a double value and receive Eigen::Array2d
   * as parameter
   */
  void initializeLevelset(std::function<double(Eigen::Array2d)> phi);
  void initializeLevelset(Shape shape);

  /**
   * @brief Given a cell Id, computes the surface location, that is, where the
   * levelset is zero. It is computed by taking the proportional distance of the
   * levelset and the cell distance to the cells coordinates.
   * If the cell is not a surface cell, than a non lethal exception is raised
   *
   * @param cellId cellId in which the surface will be computed
   * @param direction either horizontal, or vertical direction
   * @return Eigen::Array2d position of the levelset surface
   */
  Eigen::Array2d findSurfacePosition(size_t cellId, Direction direction);
  Eigen::Array2d findSurfacePosition(size_t cellId);

  /**
   * @brief Given a cell id, compute which side is the surface and return as a
   * Direction class. If the cell is not a surface, an exception is thrown
   *
   * @param cellId
   * @return Direction which the surface is, relative to the cellId position
   */
  std::vector<Direction> findSurfaceDirection(size_t cellId);

  /**
   * @brief This method is an easier way to retrieve the levelset scalar field
   * without having to rely on storing the field Id.
   *
   * @return std::vector<double>& reference vector to the scalar field id
   */
  std::vector<double>& getLevelsetField();

 protected:
  int _levelsetFieldId;
};
}  // namespace Palkia

#endif