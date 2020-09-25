#include <geometry/bounding_box.hpp>

namespace Ramuh {

BoundingBox2::BoundingBox2() : BoundingBox2(-1., 1.) {}

BoundingBox2::BoundingBox2(double min, double max)
    : BoundingBox2(Eigen::Array2d(min), Eigen::Array2d(max)) {}

BoundingBox2::BoundingBox2(Eigen::Array2d min, Eigen::Array2d max) {
  _min = min.min(max);
  _max = max.max(min);
}

Eigen::Array2d BoundingBox2::getMin() {
  return _min;
}

Eigen::Array2d BoundingBox2::getMax() {
  return _max;
}

Eigen::Array2d BoundingBox2::getCenter() {
  return (_max + _min) / 2.0;
}

Eigen::Array2d BoundingBox2::getSize() {
  return _max - _min;
}

// TODO: Check if new min or max is actually lower/bigger the existing
// counterpart
void BoundingBox2::setMin(Eigen::Array2d newMin) {
  _min = newMin;
}

void BoundingBox2::setMax(Eigen::Array2d newMax) {
  _max = newMax;
}

void BoundingBox2::setMin(double newMin) {
  setMin(Eigen::Array2d(newMin));
}

void BoundingBox2::setMax(double newMax) {
  setMax(Eigen::Array2d(newMax));
}

Eigen::Array2d BoundingBox2::clamp(Eigen::Array2d point) {
  point[0] = std::min(_max[0], std::max(_min[0], point[0]));
  point[1] = std::min(_max[1], std::max(_min[1], point[1]));
  return point;
}

bool BoundingBox2::contains(Eigen::Array2d point) {
  if ((point[0] <= _max[0] && point[1] <= _max[1]) &&
      (point[0] >= _min[0] && point[1] >= _min[1]))
    return true;
  return false;
}

bool BoundingBox2::contains(BoundingBox2 box) {
  auto bmin = box.getMin();
  auto bmax = box.getMax();
  if (contains(bmin) && contains(bmax))
    return true;
  return false;
}

BoundingBox2 BoundingBox2::unitBox() {
  return BoundingBox2(Eigen::Array2d(0, 0), Eigen::Array2d(1, 1));
}

std::vector<BoundingBox2> BoundingBox2::subdivide() {
  return subdivide(1);
}

std::vector<BoundingBox2> BoundingBox2::subdivide(int subdivisions) {
  return subdivide(subdivisions, subdivisions);
}

std::vector<BoundingBox2> BoundingBox2::subdivide(int xDivisions,
                                                  int yDivisions) {
  std::vector<BoundingBox2> divisions;
  xDivisions += 1;
  yDivisions += 1;

  Eigen::Array2d size = getSize();
  Eigen::Array2d dividedSize =
      size.cwiseQuotient(Eigen::Array2d(xDivisions, yDivisions));

  for (size_t j = 0; j < yDivisions; j++) {
    for (size_t i = 0; i < xDivisions; i++) {
      divisions.emplace_back(
          _min + dividedSize.cwiseProduct(Eigen::Array2d(i, j)),
          _min + dividedSize.cwiseProduct(Eigen::Array2d(i + 1, j + 1)));
    }
  }

  return divisions;
}

BoundingBox2::Side BoundingBox2::computeRelativePosition(Eigen::Array2d point) {
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

std::ostream& operator<<(std::ostream& output, const BoundingBox2& box) {
  output << "(" << box._min[0] << ", " << box._min[1] << ") to (";
  output << box._max[0] << ", " << box._max[1] << ")";
  return output;
}

}  // namespace Ramuh