#include <cmath>
#include <geometry/cube_tree.hpp>
#include <limits>

namespace Ramuh {

CubeTree::CubeTree() {
  _treeDepth = 5;
}

std::vector<int> CubeTree::getNodeChildren(int nodeId) {
  std::vector<int> children;
  for (size_t i = 1; i <= 8; i++) {
    children.emplace_back(nodeId * 8 + i);
  }

  return children;
}

// build the auxiliary structure on top of the mesh
CubeTree::CubeTree(const Ramuh::FaceVertexMesh& mesh) : CubeTree() {
  _mesh = std::make_shared<Ramuh::FaceVertexMesh>(mesh);

  // Compute mesh bounding box
  Ramuh::BoundingBox3 domain = _mesh->computeBoundingBox();
  domain.cubify();

  // Adding root
  _boxes.emplace_back(domain);
  std::vector<Ramuh::BoundingBox3> children = _boxes[0].subdivide();
  _boxes.insert(_boxes.end(), children.begin(), children.end());
  int startNode = 0;
  int endNode;

  // Adding childrens boxes recursively
  for (size_t level = 1; level <= _treeDepth; level++) {
    int endNode = startNode + std::pow(8, level);
    for (size_t nodeId = startNode + 1; nodeId <= endNode; nodeId++) {
      _hasVertex.push_back(false);
      children = _boxes[nodeId].subdivide();
      _boxes.insert(_boxes.end(), children.begin(), children.end());

      // For leaf nodes, set the map to hold the vertices
      if (level == _treeDepth - 1) {
        _verticesInNode[nodeId] = std::set<int>();
      }
    }
    startNode = endNode;
  }

  // Run over all vertices and add them into the nodes
  _hasVertex[0] = true;
  std::vector<Eigen::Array3f> positions = _mesh->getVerticesPosition();
  for (size_t vertexId = 0; vertexId < positions.size(); vertexId++) {
    Eigen::Array3f point = positions[vertexId];
    int nodeId = 0;
    for (size_t level = 1; level <= _treeDepth; level++) {
      std::vector<int> children = getNodeChildren(nodeId);

      // Check which children contains the vertice until reach leaves
      for (auto& childId : children) {
        if (_boxes[childId].contains(point.cast<double>())) {
          _hasVertex[childId] = true;
          nodeId = childId;
          break;
        }
      }
    }
    // And add the vertice to the leaf
    _verticesInNode[nodeId].insert(vertexId);
  }
}  // namespace Ramuh

int CubeTree::countLeavesWithVertices() {
  // Start at first leaf node until the last one
  int startNode = 8 * (pow(8, _treeDepth - 1) - 1) / (8 - 1);
  int endNode = startNode + pow(8, _treeDepth);
  int count = 0;
  for (size_t nodeId = startNode + 1; nodeId <= endNode; nodeId++) {
    if (_hasVertex[nodeId]) {
      count++;
    }
  }
  return count;
}

Eigen::Array3f CubeTree::findClosestPoint(Eigen::Array3f pointQuery,
                                          float maxDistance) {
  int nodeId = 0, closestCellId;
  float distance = std::numeric_limits<float>::max();

  // Find Closest leaf cell
  for (size_t level = 1; level <= _treeDepth; level++) {
    std::vector<int> children = getNodeChildren(nodeId);
    // Check which children that is closest to the query
    distance = std::numeric_limits<float>::max();
    for (auto& childId : children) {
      if (_boxes[childId].contains(pointQuery.cast<double>())) {
        closestCellId = childId;
        break;
      }
      if (_hasVertex[childId]) {
        float childDistance = (float)_boxes[childId].computeDistanceToPoint(
            pointQuery.cast<double>());
        if (distance > childDistance && distance <= maxDistance) {
          closestCellId = childId;
          nodeId = childId;
          distance = childDistance;
        }
      }
    }
  }

  std::vector<int> faces;
  // With the closest cell, check vertices inside it and retrieve their faces.
  for (auto& vertexId : _verticesInNode[closestCellId]) {
    std::vector<int> faceList = _mesh->getVertexFaces(vertexId);
    faces.insert(faces.end(), faceList.begin(), faceList.end());
  }

  // For all faces crossing the cell, find its closest point and compute the
  // distance
  distance = std::numeric_limits<float>::max();
  Eigen::Array3f closestPoint = pointQuery;
  for (auto&& face : faces) {
    Eigen::Array3f facePoint = _mesh->findClosestPointToFace(face, pointQuery);
    float pointDistance = (pointQuery - facePoint).matrix().norm();
    if (pointDistance <= maxDistance && pointDistance < distance) {
      distance = pointDistance;
      closestPoint = facePoint;
    }
  }

  return closestPoint;
}

}  // namespace Ramuh
