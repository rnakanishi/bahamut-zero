#include <cmath>
#include <geometry/cube_tree.hpp>
#include <limits>
#include <queue>

namespace Ramuh {

CubeTree::CubeTree() {
  _treeDepth = 5;
}

std::vector<int> CubeTree::getNodeChildren(int nodeId) const {
  std::vector<int> children;
  for (size_t i = 1; i <= 8; i++) {
    children.emplace_back(nodeId * 8 + i);
  }

  return children;
}

int CubeTree::getNodeLevel(int nodeId) const {
  int level = 0;
  int maxId = 0;
  int cumulative = 1;
  while (nodeId > maxId) {
    level++;
    cumulative = cumulative * 8;
    maxId = maxId + cumulative;
  }
  return level;
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
      _hasFace.push_back(false);
      children = _boxes[nodeId].subdivide();
      _boxes.insert(_boxes.end(), children.begin(), children.end());

      // For leaf nodes, set the map to hold the vertices
      if (level == _treeDepth - 1) {
        _verticesInNode[nodeId] = std::set<int>();
      }
    }
    startNode = endNode;
  }

  // Run over all faces and add them into the nodes
  _hasFace[0] = true;
  std::vector<Eigen::Array3f> positions = _mesh->getVerticesPositionVector();

  for (size_t faceId = 0; faceId < _mesh->getFacesCount(); faceId++) {
    std::vector<int> triangleVertices = _mesh->getFaceVertices(faceId);
    std::vector<Eigen::Array3d> triangle;
    triangle.emplace_back(
        _mesh->getVertexCoordinate(triangleVertices[0]).cast<double>());
    triangle.emplace_back(
        _mesh->getVertexCoordinate(triangleVertices[1]).cast<double>());
    triangle.emplace_back(
        _mesh->getVertexCoordinate(triangleVertices[2]).cast<double>());

    std::queue<int> nodesId;
    nodesId.push(0);  // root node
    while (!nodesId.empty()) {
      std::vector<int> children = getNodeChildren(nodesId.front());
      nodesId.pop();
      // Check which children contains the vertice until reach leaves
      for (auto& childId : children) {
        if (_boxes[childId].doesIntersectWithTriangle(triangle) ||
            _boxes[childId].contains(triangle[0]) ||
            _boxes[childId].contains(triangle[1]) ||
            _boxes[childId].contains(triangle[2])) {
          _hasFace[childId] = true;
          if (getNodeLevel(childId) < _treeDepth) {
            nodesId.push(childId);
          } else {
            // Add faces to the leaves
            _facesInNode[childId].insert(faceId);
          }
        }
      }
    }
  }
}

int CubeTree::countLeavesWithVertices() const {
  // Start at first leaf node until the last one
  int startNode = 8 * (pow(8, _treeDepth - 1) - 1) / (8 - 1);
  int endNode = startNode + pow(8, _treeDepth);
  int count = 0;
  for (size_t nodeId = startNode + 1; nodeId <= endNode; nodeId++) {
    if (_hasFace[nodeId]) {
      count++;
    }
  }
  return count;
}

Eigen::Array3f CubeTree::findClosestPoint(Eigen::Array3f pointQuery,
                                          float maxDistance) const {
  int nodeId = 0, closestCellId;
  float distance = std::numeric_limits<float>::max();

  // Find Closest leaf cell
  for (size_t level = 1; level <= _treeDepth; level++) {
    std::vector<int> children = getNodeChildren(nodeId);
    // Check which children that is closest to the query
    distance = std::numeric_limits<float>::max();
    for (auto& childId : children) {
      if (_hasFace[childId]) {
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

  // With the closest cell, retrieve all faces in that cell
  // For all faces crossing the cell, find its closest point and compute the
  // distance
  distance = std::numeric_limits<float>::max();
  Eigen::Array3f closestPoint = pointQuery;
  for (auto&& face : _facesInNode.at(closestCellId)) {
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
