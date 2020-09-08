#ifndef __PALKIA_BOUNDING_BOX_HPP__
#define __PALKIA_BOUNDING_BOX_HPP__

#include <Eigen/Dense>

namespace Palkia {

class BoundingBox2 {
 public:
  /**
   * @brief This enumeration represents the position of a certaing coordingate
   * relative to the bounding box region. If the coordinate is inside the box,
   * then it receives a INSIDE value. Otherwise, if outside, each coordinate is
   * checked and the value is given (LEFT, RIGHT, BOTTOM, TOP, DIAGONAL)
   *
   */
  enum class Side {
    INSIDE = -1,
    LEFT = 0,
    RIGHT = 1,
    BOTTOM = 2,
    TOP = 3,
    DIAGONAL = 4
  };

  /**
   * @brief Construct a new Bounding Box 2D. If points are set as inputs, they
   * are used as minimun and maximum coordinates for the bounding box. In case
   * no input is given, the Bounding Box is build from (-1, -1) to (+1, +1)
   * All box`s information are built over the coordinates provided
   *
   */
  BoundingBox2();
  BoundingBox2(Eigen::Array2d min, Eigen::Array2d max);
  BoundingBox2(double min, double max);

  /**
   * @brief creates a new instance of a bounding box with min coordinates at
   *origin and unit side (positive coordinates).
   *
   * @return BoundingBox2 instance
   **/
  static BoundingBox2 unitBox();

  /**
   * @brief Returns this bounding box sides size with double precision in a
   *Eigen::Array2d format, corresponding to horizontal side (x-axis) and
   *vertical side (y-axis)
   *
   * @return Eigen::Array2d Each coordinate of the array corresponds to the
   *respective side's size.
   **/
  Eigen::Array2d getSize();

  /**
   * @brief Get the minimum coordinate (left bottom) vertex location
   *
   * @return Eigen::Array2d coordinate
   */
  Eigen::Array2d getMin();

  /**
   * @brief Get the Max coordinate (rigth top) vertex location
   *
   * @return Eigen::Array2d coordinate
   */
  Eigen::Array2d getMax();

  /**
   * @brief Get the center coordinate of the bounding box. It is computed over
   * the average of the minimum and maximum coordinates
   *
   * @return Eigen::Array2d
   */
  Eigen::Array2d getCenter();

  /**
   * @brief Set the minimum coordinate (left bottom) of the bounding box. If a
   * dobule is given, the coordinates are duplicated
   *
   * @param newMin the nem coordinate
   */
  void setMin(Eigen::Array2d newMin);
  void setMin(double newMin);

  /**
   * @brief Set the maximum coordinate (right top) of the bounding box. If a
   * dobule is given, the coordinates are duplicated
   *
   * @param newMax new max coordinate
   */
  void setMax(Eigen::Array2d newMax);
  void setMax(double newMax);

  /**
   * @brief Given a entry point, clamps the point coordinates to match the
   * bounding box in the following way:
   *  - If the point is inside the box, nothing is done and the input coordinate
   * is returned
   *  - If x-coordinate is off the box, it is rounded to the closest box side
   * (left or right)
   *  - Same is done for the y-coordinate
   *
   * @param point to be evaluated
   * @return Eigen::Array2d point with the clamped coordinates
   */
  Eigen::Array2d clamp(Eigen::Array2d point);

  /**
   * @brief Given a point in 2D space, checks if the input is inside the
   * bounding box. This method returns true if the point is completelu inside or
   * if the point is over some of the bounding box edges
   *
   * @param point to be evaluated
   * @return true if inside
   * @return false otherwise
   */
  bool contains(Eigen::Array2d point);

  /**
   * @brief Given a bounding box, checks if the input box is completely inside
   * this box, by checking the input max and min coordinaes. If any of the
   * coordinates is outside, then a false statement is returned
   *
   * @param box to be evaluated
   * @return true if completely inside
   * @return false if any of the extremities is outside
   */
  bool contains(BoundingBox2 box);

  /**
   * @brief Subdivide this BoundingBox2 and return a vector with the new Boxes.
   *
   * If no parameter is given, then is divided once in each axis.
   * If one int is given then the subdivisions are counted following the given
   * value. If two ints are given, then they correspond to the amount of the
   * divisions in horizontal axis and the amount of divisions in the vertical
   * axis
   *
   * @return std::vector<BoundingBox2>
   */
  std::vector<BoundingBox2> subdivide();
  std::vector<BoundingBox2> subdivide(int subdivisions);
  std::vector<BoundingBox2> subdivide(int xDivisions, int yDivisions);

  /**
   * @brief Operator overload for printing the values of the boundning box. It
   * describes the lower left corner of the box and the top right corner.
   *
   * @param o
   * @param b
   * @return std::ostream&
   */
  friend std::ostream& operator<<(std::ostream& o, const BoundingBox2& b);

  /**
   * @brief This method computes the relative position of a certain point
   * relative to te hbounding box region. The value can vary according to:
   * - INSIDE: if point is inside the region of the box;
   * - LEFT: if x coordinate is lower than the center and y is between the
   * region limits;
   * - RIGHT: if x coordinate is bigger than the center and y is between the
   * region limits;
   * - BOTTOM: if y coordinate is lower than the center and x is between the
   * region limits;
   * - TOP: if y coordinate is bigger than the center and x is between the
   * region limits;
   * - Diagonal: if both coordinates are outside the region limits
   *
   * @param point
   * @return std::vector<BoundingBox2::Side> a vector of size 2 containing the
   * relative positions
   */
  BoundingBox2::Side computeRelativePosition(Eigen::Array2d point);

 private:
  Eigen::Array2d _min, _max;
};

/***************************************************************************/
/***************************************************************************/

class BoundingBox3 {
 public:
  /**
   * @brief This enumeration represents the position of a certaing coordingate
   * relative to the bounding box region. If the coordinate is inside the box,
   * then it receives a INSIDE value. Otherwise, if outside, each coordinate is
   * checked and the value is given (LEFT, RIGHT, BOTTOM, TOP, DIAGONAL)
   *
   */
  enum class Side {
    INSIDE = -1,
    LEFT = 0,
    RIGHT = 1,
    BOTTOM = 2,
    TOP = 3,
    BACK = 4,
    FRONT = 5,
    DIAGONAL = 6
  };

  /**
   * @brief Construct a new Bounding Box 2D. If points are set as inputs, they
   * are used as minimun and maximum coordinates for the bounding box. In case
   * no input is given, the Bounding Box is build from (-1, -1) to (+1, +1)
   * All box`s information are built over the coordinates provided
   *
   */
  BoundingBox3();
  BoundingBox3(Eigen::Array3d min, Eigen::Array3d max);
  BoundingBox3(double min, double max);

  /**
   * @brief creates a new instance of a bounding box with min coordinates at
   *origin and unit side (positive coordinates).
   *
   * @return BoundingBox3 instance
   **/
  static BoundingBox3 unitBox();

  /**
   * @brief Returns this bounding box sides size with double precision in a
   *Eigen::Array3d format, corresponding to horizontal side (x-axis) and
   *vertical side (y-axis)
   *
   * @return Eigen::Array3d Each coordinate of the array corresponds to the
   *respective side's size.
   **/
  Eigen::Array3d getSize();

  /**
   * @brief Get the minimum coordinate (left bottom) vertex location
   *
   * @return Eigen::Array3d coordinate
   */
  Eigen::Array3d getMin();

  /**
   * @brief Get the Max coordinate (rigth top) vertex location
   *
   * @return Eigen::Array3d coordinate
   */
  Eigen::Array3d getMax();

  /**
   * @brief Get the center coordinate of the bounding box. It is computed over
   * the average of the minimum and maximum coordinates
   *
   * @return Eigen::Array3d
   */
  Eigen::Array3d getCenter();

  /**
   * @brief Set the minimum coordinate (left bottom) of the bounding box. If a
   * dobule is given, the coordinates are duplicated
   *
   * @param newMin the nem coordinate
   */
  void setMin(Eigen::Array3d newMin);
  void setMin(double newMin);

  /**
   * @brief Set the maximum coordinate (right top) of the bounding box. If a
   * dobule is given, the coordinates are duplicated
   *
   * @param newMax new max coordinate
   */
  void setMax(Eigen::Array3d newMax);
  void setMax(double newMax);

  /**
   * @brief Given a entry point, clamps the point coordinates to match the
   * bounding box in the following way:
   *  - If the point is inside the box, nothing is done and the input coordinate
   * is returned
   *  - If x-coordinate is off the box, it is rounded to the closest box side
   * (left or right)
   *  - Same is done for the y-coordinate
   *
   * @param point to be evaluated
   * @return Eigen::Array3d point with the clamped coordinates
   */
  Eigen::Array3d clamp(Eigen::Array3d point);

  /**
   * @brief Given a point in 3D space, checks if the input is inside the
   * bounding box. This method returns true if the point is completelu inside or
   * if the point is over some of the bounding box edges
   *
   * @param point to be evaluated
   * @return true if inside
   * @return false otherwise
   */
  bool contains(Eigen::Array3d point);

  /**
   * @brief Given a bounding box, checks if the input box is completely inside
   * this box, by checking the input max and min coordinaes. If any of the
   * coordinates is outside, then a false statement is returned
   *
   * @param box to be evaluated
   * @return true if completely inside
   * @return false if any of the extremities is outside
   */
  bool contains(BoundingBox3 box);

  /**
   * @brief Subdivide this BoundingBox3 and return a vector with the new Boxes.
   *
   * If no parameter is given, then is divided once in each axis.
   * If one int is given then the subdivisions are counted following the given
   * value. If two ints are given, then they correspond to the amount of the
   * divisions in horizontal axis and the amount of divisions in the vertical
   * axis
   *
   * @return std::vector<BoundingBox3>
   */
  std::vector<BoundingBox3> subdivide();
  std::vector<BoundingBox3> subdivide(int subdivisions);
  std::vector<BoundingBox3> subdivide(int xDivisions,
                                      int yDivisions,
                                      int zDivisions);

  /**
   * @brief Operator overload for printing the values of the boundning box. It
   * describes the lower left corner of the box and the top right corner.
   *
   * @param o
   * @param b
   * @return std::ostream&
   */
  friend std::ostream& operator<<(std::ostream& o, const BoundingBox3& b);

  /**
   * @brief This method computes the relative position of a certain point
   * relative to te hbounding box region. The value can vary according to:
   * - INSIDE: if point is inside the region of the box;
   * - LEFT: if x coordinate is lower than the center and y is between the
   * region limits;
   * - RIGHT: if x coordinate is bigger than the center and y is between the
   * region limits;
   * - BOTTOM: if y coordinate is lower than the center and x is between the
   * region limits;
   * - TOP: if y coordinate is bigger than the center and x is between the
   * region limits;
   * - Diagonal: if both coordinates are outside the region limits
   *
   * @param point
   * @return std::vector<BoundingBox3::Side> a vector of size 3 containing the
   * relative positions
   */
  BoundingBox3::Side computeRelativePosition(Eigen::Array3d point);

 private:
  Eigen::Array3d _min, _max;
};
}  // namespace Palkia

#endif