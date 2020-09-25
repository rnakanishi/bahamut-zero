#include <geometry/face_vertex_mesh.hpp>
#include <limits>

namespace Ramuh {
FaceVertexMesh::FaceVertexMesh() {}

int FaceVertexMesh::addVertex(Eigen::Array3f point) {
  _verticesPosition.emplace_back(point);
  _verticesFaces.emplace_back(std::set<int>());
  _bbox.mergeBoxes(point.cast<double>());

  return _verticesPosition.size() - 1;
}

int FaceVertexMesh::addNormal(Eigen::Vector3f normal) {
  _normals.emplace_back(normal);
  return _normals.size() - 1;
}

int FaceVertexMesh::addFace(Eigen::ArrayXi facePoints) {
  int faceIndex = _faceVertices.size();
  _faceVertices.emplace_back(std::set<int>());
  for (size_t i = 0; i < facePoints.size(); i++) {
    int vertexId = facePoints[i];
    _faceVertices[faceIndex].insert(vertexId);
    _verticesFaces[vertexId].insert(faceIndex);
  }
  return faceIndex;
}

int FaceVertexMesh::getVerticesCount() {
  return _verticesPosition.size();
}

int FaceVertexMesh::getNormalsCount() {
  return _normals.size();
}

int FaceVertexMesh::getFacesCount() {
  return _faceVertices.size();
}

std::vector<int> FaceVertexMesh::getVertexFaces(int vertexId) {
  return std::vector<int>(_verticesFaces[vertexId].begin(),
                          _verticesFaces[vertexId].end());
}

std::vector<int> FaceVertexMesh::getFaceVertices(int faceId) {
  return std::vector<int>(_faceVertices[faceId].begin(),
                          _faceVertices[faceId].end());
}

Eigen::Array3f FaceVertexMesh::getVertexCoordinate(int vertexId) {
  return _verticesPosition[vertexId];
}

const BoundingBox3& FaceVertexMesh::computeBoundingBox() {
  return _bbox;
}

const std::vector<Eigen::Array3f>& FaceVertexMesh::getVerticesPosition() {
  return _verticesPosition;
}

bool FaceVertexMesh::doesFaceContainPoint(int faceId, Eigen::Array3f point) {
  std::vector<int> vertices = getFaceVertices(faceId);
  float dotProd =
      (point - _verticesPosition[vertices[0]])
          .matrix()
          .dot((_verticesPosition[vertices[1]] - _verticesPosition[vertices[0]])
                   .matrix());

  if (dotProd * (point - _verticesPosition[vertices[1]])
                    .matrix()
                    .dot((_verticesPosition[vertices[2]] -
                          _verticesPosition[vertices[1]])
                             .matrix()) <
      0)
    return false;

  if (dotProd * (point - _verticesPosition[vertices[2]])
                    .matrix()
                    .dot((_verticesPosition[vertices[0]] -
                          _verticesPosition[vertices[2]])
                             .matrix()) <
      0)
    return false;
  return true;
}

float FaceVertexMesh::computeDistanceToFace(int faceId, Eigen::Array3f point) {
  float distance = std::numeric_limits<float>::max();

  // get one vertex from the face to form a PV vector
  std::vector<int> vertices = getFaceVertices(faceId);
  Eigen::Vector3f vectorPV = point - _verticesPosition[vertices[0]];

  // Compute face Normal (cross product) and normalize
  Eigen::Vector3f normal =
      Eigen::Vector3f(_verticesPosition[vertices[1]] -
                      _verticesPosition[vertices[0]])
          .cross(Eigen::Vector3f(_verticesPosition[vertices[2]] -
                                 _verticesPosition[vertices[0]]));
  normal.normalize();

  // Project PV to nonrmal
  distance = vectorPV.dot(normal);
  Eigen::Vector3f projection = -distance * normal;

  // Check if face contains projection
  if (doesFaceContainPoint(faceId, projection))
    return abs(distance);

  // Project into edges

  return std::abs(distance);
}

Eigen::Array3f FaceVertexMesh::findClosestPointToFace(int faceId,
                                                      Eigen::Array3f point) {
  std::vector<int> vertices = getFaceVertices(faceId);
  Eigen::Array3f vertexA = _verticesPosition[vertices[0]];
  Eigen::Array3f vertexB = _verticesPosition[vertices[1]];
  Eigen::Array3f vertexC = _verticesPosition[vertices[2]];

  // Check if P is in vertex region outside of A
  Eigen::Vector3f ab = (vertexB - vertexA).matrix();
  Eigen::Vector3f ac = (vertexC - vertexA).matrix();
  Eigen::Vector3f ap = (point - vertexA).matrix();
  float d1 = ab.dot(ap);
  float d2 = ac.dot(ap);
  if (d1 <= 0 && d2 <= 0) {
    return vertexA;
  }

  // Check for B now
  Eigen::Vector3f bp = (point - vertexB).matrix();
  float d3 = ab.dot(bp);
  float d4 = ac.dot(bp);
  if (d3 >= 0 && d3 <= d3) {
    return vertexB;
  }

  // If P in edge regino of Ab, return the projection point
  float vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    return vertexA + (d1 / (d1 - d3)) * ab.array();
  }

  // Check if P is in vertex region outside of C
  Eigen::Vector3f cp = point - vertexC;
  float d5 = ab.dot(cp);
  float d6 = ac.dot(cp);
  if (d6 >= 0 && d5 <= d6) {
    return vertexC;
  }

  // if p in edge AC, return the projection onto AC
  float vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    return vertexA + (d2 / (d2 - d6)) * ac.array();
  }

  // Check for edge region BC
  float va = d3 * d6 - d5 * d4;
  if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    return vertexB +
           ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (vertexC - vertexB);
  }

  // If reach here, then P is inside face
  float d = 1.0 / (va + vb + vc);
  return vertexA + ab.array() * (vb * d) + ac.array() * (vc * d);
}

}  // namespace Ramuh
