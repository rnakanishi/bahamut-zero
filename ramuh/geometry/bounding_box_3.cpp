#include <geometry/bounding_box.hpp>

namespace Ramuh {

BoundingBox3::BoundingBox3() : BoundingBox3(-1., 1.) {}

BoundingBox3::BoundingBox3(double min, double max)
    : BoundingBox3(Eigen::Array3d(min), Eigen::Array3d(max)) {}

BoundingBox3::BoundingBox3(Eigen::Array3d min, Eigen::Array3d max) {
  _min = min.min(max);
  _max = max.max(min);
}

Eigen::Array3d BoundingBox3::getMin() const {
  return _min;
}

Eigen::Array3d BoundingBox3::getMax() const {
  return _max;
}

Eigen::Array3d BoundingBox3::getCenter() const {
  return (_max + _min) / 2.0;
}

Eigen::Array3d BoundingBox3::getSize() const {
  return _max - _min;
}

// TODO: Check if new min or max is actually lower/bigger the existing
//     counterpart
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

bool BoundingBox3::contains(Eigen::Array3d point) const {
  if ((point[0] <= _max[0] && point[1] <= _max[1] && point[2] <= _max[2]) &&
      (point[0] >= _min[0] && point[1] >= _min[1] && point[2] >= _min[2]))
    return true;
  return false;
}

bool BoundingBox3::contains(BoundingBox3 box) const {
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
  zDivisions += 1;

  Eigen::Array3d size = getSize();
  Eigen::Array3d dividedSize =
      size.cwiseQuotient(Eigen::Array3d(xDivisions, yDivisions, zDivisions));

  for (size_t k = 0; k < zDivisions; k++) {
    for (size_t j = 0; j < yDivisions; j++) {
      for (size_t i = 0; i < xDivisions; i++) {
        divisions.emplace_back(
            _min + dividedSize.cwiseProduct(Eigen::Array3d(i, j, k)),
            _min +
                dividedSize.cwiseProduct(Eigen::Array3d(i + 1, j + 1, k + 1)));
      }
    }
  }

  return divisions;
}

BoundingBox3::Side BoundingBox3::computeRelativePosition(Eigen::Array3d point) {
  // TODO: Translate this to 3D
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

BoundingBox3 BoundingBox3::mergeBoxes(BoundingBox3 box) {
  mergeBoxes(box.getMin());
  return mergeBoxes(box.getMax());
}

BoundingBox3 BoundingBox3::mergeBoxes(Eigen::Array3d point) {
  if (!contains(point)) {
    for (size_t i = 0; i < 3; i++) {
      if (_min[i] > point[i])
        _min[i] = point[i];
      else if (_max[i] < point[i])
        _max[i] = point[i];
    }
  }

  return *this;
}

void BoundingBox3::cubify() {
  Eigen::Array3d size = getSize(), center = getCenter();
  double biggestSize = size.maxCoeff();
  setMin(center - biggestSize / 2.0);
  setMax(center + biggestSize / 2.0);
}

double BoundingBox3::computeDistanceToPoint(Eigen::Array3d point) const {
  if (contains(point))
    return 0.0;

  // Align with (-1,-,1-,1) (1,1,1)
  point = point - getCenter();
  point = point / (getSize() / 2.);

  // distance = sqrt( sum( max( abs(x) - 1) ) )
  Eigen::Vector3d pointVector = point.matrix();
  pointVector = pointVector.cwiseAbs() - Eigen::Vector3d(1, 1, 1);
  pointVector = pointVector.cwiseMax(Eigen::Vector3d(0, 0, 0));

  pointVector = pointVector.cwiseProduct((getSize().matrix() / 2.0));

  return pointVector.norm();
}

bool BoundingBox3::_performSatTestCategory3(
    std::vector<Eigen::Array3d> triangle,
    Eigen::Vector3d axis) const {
  Eigen::Array3d extent((_max - _min) * 0.5);
  std::vector<Eigen::Vector3d> normals;  // Box normals
  Eigen::Array3d p;
  normals.emplace_back(1, 0, 0);
  normals.emplace_back(0, 1, 0);
  normals.emplace_back(0, 0, 1);
  double r = 0;
  for (size_t i = 0; i < 3; i++) {
    r += extent[i] * std::abs(normals[i].dot(axis));
    p[i] = triangle[i].matrix().dot(axis);
  }
  return (std::max(-p.maxCoeff(), p.minCoeff()) > r);
}

bool BoundingBox3::doesIntersectWithTriangle(
    std::vector<Eigen::Array3d> triangle) const {
  Eigen::Array3d center = getCenter();
  // Translate box to origin
  for (auto& vertex : triangle) {
    vertex = vertex - center;
  }
  Eigen::Array3d extent((_max - _min) * 0.5);

  // Test category 3: axis from cross products combination of both edges
  std::vector<Eigen::Vector3d> normals;  // Box normals
  normals.emplace_back(1, 0, 0);
  normals.emplace_back(0, 1, 0);
  normals.emplace_back(0, 0, 1);
  std::vector<Eigen::Vector3d> edges;  // Triangle edges
  edges.emplace_back(Eigen::Vector3d(triangle[0] - triangle[1]));
  edges.emplace_back(Eigen::Vector3d(triangle[1] - triangle[2]));
  edges.emplace_back(Eigen::Vector3d(triangle[2] - triangle[0]));
  for (auto& normal : normals) {
    for (auto& edge : edges) {
      if (_performSatTestCategory3(triangle, normal.cross(edge)))
        return false;
    }
  }

  // Test category 1: box face normals
  for (size_t i = 0; i < 3; i++) {
    double coordMax =
        std::max(std::max(normals[0][i], normals[1][i]), normals[2][i]);
    for (size_t i = 0; i < 3; i++) {
      double coordMin =
          std::min(std::min(normals[0][i], normals[1][i]), normals[2][i]);
      if (coordMax < -extent[i] || coordMin > extent[i]) {
        return false;
      }
    }
  }

  // Test category 2: triangle face normal
  Eigen::Vector3d tNormal = edges[0].cross(edges[1]).normalized();
  double r = extent.matrix().dot(tNormal.cwiseAbs());
  double distPlane = -tNormal.dot(triangle[0].matrix());
  return std::abs(distPlane) <= r;
}

}  // namespace Ramuh