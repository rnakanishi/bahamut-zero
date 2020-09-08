#include <grid/cell_graph2.hpp>
#include <map>
#include <utils/exception.hpp>

namespace Palkia {
CellGraph2::CellGraph2() : CellGraph2::CellGraph2(BoundingBox2(0, 1)) {}

CellGraph2::CellGraph2(BoundingBox2 region, int maxLevel) : _nodes(region) {
  connectNodes(0, 1);
}

void CellGraph2::connectNodes(size_t nodeA, size_t nodeB) {
  size_t newEdge = _edges.createEdge(nodeA, nodeB);
  _nodes.addEdge(nodeA, newEdge);
  _nodes.addEdge(nodeB, newEdge);
}

void CellGraph2::destroyNode(size_t nodeId) {
  std::list<size_t> nodeEdges = _nodes.getNodeEdges(nodeId);

  for (auto &&edge : nodeEdges) {
    destroyEdge(edge);
  }
  _nodes.destroyNode(nodeId);
}

void CellGraph2::destroyEdge(size_t edgeId) {
  std::vector<size_t> edgeNodes = _edges.getEdgeNodes(edgeId);
  _nodes.removeEdge(edgeNodes[0], edgeId);
  _nodes.removeEdge(edgeNodes[1], edgeId);

  _edges.destroyEdge(edgeId);
}

void CellGraph2::connectChildrenToParentNeighbors(
    std::vector<size_t> childrenIds, std::vector<size_t> neighbors,
    BoundingBox2 parentRegion) {
  int parentLevel = _nodes.getNodeLevel(childrenIds[0]) - 1;
  size_t edgeId;

  std::map<BoundingBox2::Side, std::vector<size_t>> sideChildren;
  sideChildren[BoundingBox2::Side::LEFT] = {0, 2};
  sideChildren[BoundingBox2::Side::RIGHT] = {1, 3};
  sideChildren[BoundingBox2::Side::BOTTOM] = {0, 1};
  sideChildren[BoundingBox2::Side::TOP] = {2, 3};

  for (size_t i = 0; i < neighbors.size(); i++) {
    BoundingBox2::Side neighborSide = parentRegion.computeRelativePosition(
        _nodes.getNodeCenterPosition(neighbors[i]));
    for (auto &&side : sideChildren) { // For all possible sidess
      if (neighborSide == side.first) {
        if (_nodes.getNodeLevel(neighbors[i]) <= parentLevel) {
          // If parent level is equal to the neighbor level, normally connect
          // the children to that neighbor
          for (auto &&childId : side.second) {
            connectNodes(childrenIds[childId], neighbors[i]);
          }
        } else {
          // In this case, children should not connect to diagonal neighbors
          for (auto &&childId : side.second) {
            neighborSide = _nodes.getNodeRegion(childrenIds[childId])
                               .computeRelativePosition(
                                   _nodes.getNodeCenterPosition(neighbors[i]));
            if (neighborSide != BoundingBox2::Side::DIAGONAL) {
              connectNodes(childrenIds[childId], neighbors[i]);
            }
          }
        }
      }
    }
  }
}

std::vector<size_t> CellGraph2::findNodeNeighbors(size_t nodeId) {
  std::list<size_t> &nodeEdges = _nodes.getNodeEdges(nodeId);
  std::vector<size_t> neighbors;

  for (auto &&edge : nodeEdges) {
    if (_edges.isValid(edge)) {
      auto edgeNodes = _edges.getEdgeNodes(edge);
      if (edgeNodes[0] != nodeId) {
        neighbors.push_back(edgeNodes[0]);
      } else {
        neighbors.push_back(edgeNodes[1]);
      }
    }
  }
  return neighbors;
}

std::vector<size_t> CellGraph2::refineNode(size_t nodeId) {
  if (nodeId >= _nodes.getCellsCount()) {
    throw(Arceus::UnexpectedParameterException(306, "CellGraph2::refineNode"));
  }

  std::vector<size_t> neighbors = findNodeNeighbors(nodeId);
  BoundingBox2 parentRegion = _nodes.getNodeRegion(nodeId);
  std::vector<size_t> childrenIds = _nodes.refineNode(nodeId);

  // Set the Address code of the new Node
  auto parentLevel = _nodes.getNodeLevel(nodeId);
  size_t indexStep = 1u << (getMaxLevel() - (parentLevel + 1));
  Eigen::Array2i parentAddressCode = _nodes.getNodeAddressCode(nodeId);
  _nodes.setNodeAddressCode(
      childrenIds[0],
      Eigen::Array2i(parentAddressCode[0], parentAddressCode[1]));
  _nodes.setNodeAddressCode(
      childrenIds[1],
      Eigen::Array2i(parentAddressCode[0] + indexStep, parentAddressCode[1]));
  _nodes.setNodeAddressCode(
      childrenIds[2],
      Eigen::Array2i(parentAddressCode[0], parentAddressCode[1] + indexStep));
  _nodes.setNodeAddressCode(childrenIds[3],
                            Eigen::Array2i(parentAddressCode[0] + indexStep,
                                           parentAddressCode[1] + indexStep));

  // Interconnect children
  connectNodes(childrenIds[0], childrenIds[1]);
  connectNodes(childrenIds[0], childrenIds[2]);
  connectNodes(childrenIds[1], childrenIds[3]);
  connectNodes(childrenIds[2], childrenIds[3]);

  // TODO: Connect new nodes to the boundary cell, so the boundary will be well
  // defined

  connectChildrenToParentNeighbors(childrenIds, neighbors, parentRegion);

  destroyNode(nodeId);

  return childrenIds;
}

int CellGraph2::getMaxLevel() { return _maxLevel; }

bool CellGraph2::canCoarse(std::vector<size_t> nodesId) {
  // Minimum family size requirement
  if (nodesId.size() != 4)
    return false;
  size_t level = _nodes.getNodeLevel(nodesId[0]);
  size_t indexStep = 1u << (_maxLevel - (level - 1u));
  Eigen::Array2i nodeFamily = _nodes.getNodeAddressCode(nodesId[0]) / indexStep;
  for (auto id : nodesId) {
    if (_nodes.getNodeLevel(id) != level)
      return false;

    if (nodeFamily.matrix() ==
        (_nodes.getNodeAddressCode(id) / indexStep).matrix())
      return false;
  }
  return true;
}

size_t CellGraph2::coarseNode(size_t nodeId) {

  std::vector<size_t> siblings = _nodes.findNodeSiblings(nodeId);
  // Minimum family size requirement
  if (siblings.size() != 4)
    return -1;

  // Sort the sibling
  return -1;
}

} // namespace Palkia
