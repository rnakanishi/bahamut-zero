#ifndef __RAMUH_CELL_CENTERED_GRID_HPP__
#define __RAMUH_CELL_CENTERED_GRID_HPP__

#include <Eigen/Dense>
#include <geometry/bounding_box.hpp>
#include <map>
#include <string>
#include <utils/exception.hpp>
#include <vector>

namespace Ramuh {

class CellCenteredGrid2 {
 public:
  /**
   * @brief Construct a new Cell Centered Grid 2 object. If no parameter is
   * given, the domain is set to the default value (-1, -1) to (1, 1) and the
   * grid size (or resolution) is set to (32, 32)
   *
   *  These values can be changed later in the code, but be careful to do that
   * before setting any information in the structure
   */
  CellCenteredGrid2();
  CellCenteredGrid2(BoundingBox2 domain, Eigen::Array2i gridSize);

  /**
   * @brief Set the Grid Size in number of cells in each coordinate (x,y)
   *
   * If any data is set before this method is called, a warning is raised,
   *because the structure can't guarantee correct access to previous existing
   *information.
   *
   * @param sizeX new number of cells in X coordinate
   * @param sizeY new number of cells in Y coordinate
   **/
  void setGridSize(int sizeX, int sizeY);
  void setGridSize(Eigen::Array2i size);

  /**
   * @brief Change the domain of the simulation. Either a new bounding box or a
   * set of min and max coordinates can be given.
   *
   * If any data is set before this method is called, a warning is raised,
   *because the structure can't guarantee correct access to previous existing
   *information.
   *
   * @param domain new coordinates of the domain
   */
  void setDomain(BoundingBox2 domain);
  void setDomain(Eigen::Array2d min, Eigen::Array2d max);

  /**
   * @brief From a pair of coordinate values (i,j), computes the single id value
   *using the grid size provided ealier
   *
   * @param i x coordinate value
   * @param j y coordinate value
   * @return size_t corresponding  id value
   **/
  size_t getId(size_t i, size_t j);

  /**
   * @brief Given an id value, computes the pair (i,j) of values that
   *generated it according to domain size
   *
   * @param id cell Id
   * @return std::vector containing (i,j) pair
   **/
  std::vector<size_t> getIj(size_t id);

  /**
   * @brief Given a cell Id, compute and return all existing cells id in a
   * 4-neighborhood. This method takes into account cell Id that is in domain
   * boundary, so the neighbor would not exist. Therefore, those non-existing
   * ids are not returned.
   * If an invalid cell id is given as input, an exception is thrown.
   *
   * @param cellId central cell id
   * @return std::vector<size_t> 4-neighborhood cells id
   */
  std::vector<size_t> getNeighborCellsId(size_t cellId);

  /**
   * @brief Get the Grid Size for each dimension, i.e., the number of cells
   *
   * @return Eigen::Array2i
   */
  Eigen::Array2i getGridSize();

  /**
   * @brief Get the resolution for the grid, i.e., the number of the cells in
   * each dimension.
   *
   * @return Eigen::Array2i
   */
  Eigen::Array2i getResolution();

  /**
   * @brief Get the Domain Bounding box in which the grid is set.
   *
   * @return BoundingBox2
   */
  BoundingBox2 getDomain();

  /**
   * @brief Compute the spacing between two cell centers for the two dimensions
   * and return it as an Eigen::Array2d. The two values may be differ,  since
   * the grid may also have different scales and different number of cells in
   * each dimension
   *
   * @return Eigen::Array2d Spacing between cells in each dimension
   */
  Eigen::Array2d getH();

  /**
   * @brief Compute the spacing between two cell centers for the two dimensions
   * and return it as an Eigen::Array2d. The two values may be differ,  since
   * the grid may also have different scales and different number of cells in
   * each dimension
   *
   * @return Eigen::Array2d Spacing between cells in each dimension
   */
  Eigen::Array2d getSpacing();

  /**
   * @brief Given a cell (i, j) or the cellId, computes the cell location in
   * space and return as a Eigen::Array2d
   *
   * @param i grid coordinate or the cellId
   * @param j grid coordinate
   * @return Eigen::Array2d cell center position in space
   */
  Eigen::Array2d getCellPosition(int i, int j);
  Eigen::Array2d getCellPosition(int id);

  /**
   * @brief Given a cell coordinate (i, j) or the cellId, compute the cell size
   * and return it in the form of a Bounding Box, containing the limits of the
   * cell.
   *
   * @param i grid coordinate, or the cellId
   * @param j grid coordinate
   * @return BoundingBox2
   */
  BoundingBox2 getCellBoundingBox(int i, int j);
  BoundingBox2 getCellBoundingBox(int id);

  /**
   * @brief Count the total number of cells by multiplying the number of cells
   * in each dimension.
   *
   * @return int total number of cells
   */
  int getCellCount();

  /**
   * @brief Create a new label in the structure for a Scalar Field. A initial
   *value for the entire grid can be assigned as well, if the parameter is
   *provided. If no initial value is given, then all values are set to zero.
   * If label already exists, then the internal id of that label is returned,
   *intead of creating a new one
   *
   * @param label for semantic purposes. Labels have to be unique over the same
   *instance
   * @return size_t return id of the label.
   **/
  size_t createCellScalarField(std::string label);
  size_t createCellScalarField(std::string label, double initialValue);

  /**
   * @brief Create a new label in the structure for a Vector Field. A initial
   *value for the entire grid can be assigned as well, if the parameter is
   *provided. If no initial value is given, then all values are set to
   *Eigen::Array2d(0, 0).
   * If label already exists, then the internal id of that label is returned,
   *intead of creating a new one
   *
   * @param label for semantic purposes. Labels have to be unique over the same
   *instance
   * @return size_t return id of the label.
   **/
  size_t createCellVectorField(std::string label);
  size_t createCellVectorField(std::string label, Eigen::Vector2d initialValue);

  /**
   * @brief Get the vector object for a given label. This vector contains whole
   *data for that label.
   *
   * @param label string value. Must have been created
   * @return std::vector<double>& vector containing the data for that label
   **/
  std::vector<double>& getCellScalarField(std::string label);
  std::vector<double>& getCellScalarField(size_t id);

  std::vector<Eigen::Vector2d>& getCellVectorField(std::string label);
  std::vector<Eigen::Vector2d>& getCellVectorField(int id);

  /**
   * @brief Given a cell field id and a point inside the domain, computes an
   * inteprolated value at that position using bilinear method. If the point is
   * outside the domain, an extrapolated (and not accurate) value is returned
   *
   * @param dataId cell field which value is wanted
   * @param position point in the space as target
   * @return double interpolated value at position
   */
  double interpolateCellScalarField(int dataId, Eigen::Array2d position);
  double interpolateCellScalarField(std::string label, Eigen::Array2d position);

  /**
   * @brief Given a cell field id and a point inside the domain, computes an
   * inteprolated Vector value at that position using bilinear method. If the
   * point is outside the domain, an extrapolated (and not accurate) value is
   * returned
   *
   * @param dataId cell field which value is wanted
   * @param position point in the space as target
   * @return double interpolated value at position
   */
  Eigen::Vector2d interpolateCellVectorField(int dataId,
                                             Eigen::Array2d position);
  Eigen::Vector2d interpolateCellVectorField(std::string label,
                                             Eigen::Array2d position);

 protected:
  Eigen::Array2i _gridSize;
  BoundingBox2 _domain;

  std::vector<std::vector<double>> _cellScalarFields;
  std::vector<std::vector<Eigen::Vector2d>> _cellVectorFields;
  std::map<std::string, size_t> _cellScalarLabels, _cellVectorLabels;
};

}  // namespace Ramuh

#endif