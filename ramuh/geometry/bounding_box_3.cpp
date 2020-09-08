#include <geometry/bounding_box.hpp>

namespace Palkia {

BoundingBox3::BoundingBox3() : BoundingBox3(-1., 1.) {}

BoundingBox3::BoundingBox3(double min, double max)
    : BoundingBox3(Eigen::Array3d(min), Eigen::Array3d(max)) {}

BoundingBox3::BoundingBox3(Eigen::Array3d min, Eigen::Array3d max) {
  _min = min;
  _max = max;
}

Eigen::Array3d BoundingBox3::getMin() {
  return _min;
}

Eigen::Array3d BoundingBox3::getMax() {
  return _max;
}

Eigen::Array3d BoundingBox3::getCenter() {
  return (_max + _min) / 2.0;
}

Eigen::Array3d BoundingBox3::getSize() {
  return _max - _min;
}

// TODO: Check if new min or max is actually lower/bigger the existing
// counterpart
void BoundingBox3::setMin(Eigen::Array3d newMin) {
  _min = newMin;
}

void BoundingBox3::setMax(Eigen::Array3d newMax) {
  _max = newMax;
}

void BoundingBox3::setMin(double newMin) {
  setMin(Eigen::Array3d(newMin));
}

void BoundingBox3::setMax(double newMax) {
  setMax(Eigen::Array3d(newMax));
}

Eigen::Array3d BoundingBox3::clamp(Eigen::Array3d point) {
  point[0] = std::min(_max[0], std::max(_min[0], point[0]));
  point[1] = std::min(_max[1], std::max(_min[1], point[1]));
  return point;
}

bool BoundingBox3::contains(Eigen::Array3d point) {
  if ((point[0] <= _max[0] && point[1] <= _max[1] && point[2] <= _max[2]) &&
      (point[0] >= _min[0] && point[1] >= _min[1] && point[2] >= _min[2]))
    return true;
  return false;
}

bool BoundingBox3::contains(BoundingBox3 box) {
  auto bmin = box.getMin();
  auto bmax = box.getMax();
  if (contains(bmin) && contains(bmax))
    return true;
  return false;
}

BoundingBox3 BoundingBox3::unitBox() {
  return BoundingBox3(Eigen::Array3d(0, 0), Eigen::Array3d(1, 1));
}

std::vector<BoundingBox3> BoundingBox3::subdivide() {
  return subdivide(1);
}

std::vector<BoundingBox3> BoundingBox3::subdivide(int subdivisions) {
  return subdivide(subdivisions, subdivisions, subdivisions);
}

std::vector<BoundingBox3> BoundingBox3::subdivide(int xDivisions,
                                                  int yDivisions,
                                                  int zDivisions) {
  std::vector<BoundingBox3> divisions;
  xDivisions += 1;
  yDivisions += 1;

  Eigen::Array3d size = getSize();
  Eigen::Array3d dividedSize =
      size.cwiseQuotient(Eigen::Array3d(xDivisions, yDivisions));

  for (size_t j = 0; j < yDivisions; j++) {
    for (size_t i = 0; i < xDivisions; i++) {
      divisions.emplace_back(
          _min + dividedSize.cwiseProduct(Eigen::Array3d(i, j)),
          _min + dividedSize.cwiseProduct(Eigen::Array3d(i + 1, j + 1)));
    }
  }

  return divisions;
}

BoundingBox3::Side BoundingBox3::computeRelativePosition(Eigen::Array3d point) {
  if (contains(point))
    return Side::INSIDE;
  if (point[0] < _min[0] && point[1] >= _min[1] && point[1] <= _max[1])
    return Side::LEFT;
  if (point[0] > _min[0] && point[1] >= _min[1] && point[1] <= _max[1])
    return Side::RIGHT;
  if (point[1] < _min[1] && point[0] >= _min[0] && point[0] <= _max[0])
    return Side::BOTTOM;
  if (point[1] > _min[1] && point[0] >= _min[0] && point[0] <= _max[0])
    return Side::TOP;
  return Side::DIAGONAL;
}

std::ostream& operator<<(std::ostream& output, const BoundingBox3& box) {
  output << "(" << box._min[0] << ", " << box._min[1] << ") to (";
  output << box._max[0] << ", " << box._max[1] << ")";
  return output;
}

}  // namespace Palkia