#include <exceptions/exception.hpp>
#include <grid/cell_graph3.hpp>
#include <map>

namespace Ramuh {

CellGraph3::CellGraph3() : CellGraph3::CellGraph3(BoundingBox3(0, 1)) {}

CellGraph3::CellGraph3(BoundingBox3 region, int maxLevel) : _nodes(region) {
  connectNodes(0, 1);
}

void CellGraph3::connectNodes(size_t nodeA, size_t nodeB) {
  size_t newEdge = _edges.createEdge(nodeA, nodeB);
  _nodes.addEdge(nodeA, newEdge);
  _nodes.addEdge(nodeB, newEdge);
}

void CellGraph3::destroyNode(size_t nodeId) {
  std::list<size_t> nodeEdges = _nodes.getNodeEdges(nodeId);

  for (auto&& edge : nodeEdges) {
    destroyEdge(edge);
  }
  _nodes.destroyNode(nodeId);
}

void CellGraph3::destroyEdge(size_t edgeId) {
  std::vector<size_t> edgeNodes = _edges.getEdgeNodes(edgeId);
  _nodes.removeEdge(edgeNodes[0], edgeId);
  _nodes.removeEdge(edgeNodes[1], edgeId);

  _edges.destroyEdge(edgeId);
}

// TODO: Check 3d node children connection to neighbors
void CellGraph3::connectChildrenToParentNeighbors(
    std::vector<size_t> childrenIds,
    std::vector<size_t> neighbors,
    BoundingBox3 parentRegion) {
  int parentLevel = _nodes.getNodeLevel(childrenIds[0]) - 1;
  size_t edgeId;

  std::map<BoundingBox3::Side, std::vector<size_t>> sideChildren;
  sideChildren[BoundingBox3::Side::LEFT] = {0, 2, 4, 6};
  sideChildren[BoundingBox3::Side::RIGHT] = {1, 3, 5, 7};
  sideChildren[BoundingBox3::Side::BOTTOM] = {0, 1, 4, 5};
  sideChildren[BoundingBox3::Side::TOP] = {2, 3, 6, 7};

  for (size_t i = 0; i < neighbors.size(); i++) {
    BoundingBox3::Side neighborSide = parentRegion.computeRelativePosition(
        _nodes.getNodeCenterPosition(neighbors[i]));
    for (auto&& side : sideChildren) {  // For all possible sidess
      if (neighborSide == side.first) {
        if (_nodes.getNodeLevel(neighbors[i]) <= parentLevel) {
          // If parent level is equal to the neighbor level, normally connect
          // the children to that neighbor
          for (auto&& childId : side.second) {
            connectNodes(childrenIds[childId], neighbors[i]);
          }
        } else {
          // In this case, children should not connect to diagonal neighbors
          for (auto&& childId : side.second) {
            neighborSide = _nodes.getNodeRegion(childrenIds[childId])
                               .computeRelativePosition(
                                   _nodes.getNodeCenterPosition(neighbors[i]));
            if (neighborSide != BoundingBox3::Side::DIAGONAL) {
              connectNodes(childrenIds[childId], neighbors[i]);
            }
          }
        }
      }
    }
  }
}

std::vector<size_t> CellGraph3::findNodeNeighbors(size_t nodeId) {
  std::list<size_t>& nodeEdges = _nodes.getNodeEdges(nodeId);
  std::vector<size_t> neighbors;

  for (auto&& edge : nodeEdges) {
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

int CellGraph3::getMaxLevel() {
  return _maxLevel;
}

bool CellGraph3::canCoarse(std::vector<size_t> nodesId) {
  // Minimum family size requirement
  if (nodesId.size() != 8)
    return false;
  size_t level = _nodes.getNodeLevel(nodesId[0]);
  size_t indexStep = 1u << (_maxLevel - (level - 1u));
  Eigen::Array3i nodeFamily = _nodes.getNodeAddressCode(nodesId[0]) / indexStep;
  for (auto id : nodesId) {
    if (_nodes.getNodeLevel(id) != level)
      return false;

    if (nodeFamily.matrix() ==
        (_nodes.getNodeAddressCode(id) / indexStep).matrix())
      return false;
  }
  return true;
}

size_t CellGraph3::coarseNode(size_t nodeId) {
  std::vector<size_t> siblings = _nodes.findNodeSiblings(nodeId);
  // Minimum family size requirement
  if (siblings.size() != 8)
    return -1;

  // Sort the sibling
  // TODO: Implemente coarseNoode 3D
  return -1;
}

std::vector<size_t> CellGraph3::refineNode(size_t nodeId) {
  if (nodeId >= _nodes.getCellsCount()) {
    throw(Bahamut::UnexpectedParameterException(306, "CellGraph3::refineNode"));
  }

  std::vector<size_t> neighbors = findNodeNeighbors(nodeId);
  BoundingBox3 parentRegion = _nodes.getNodeRegion(nodeId);
  std::vector<size_t> childrenIds = _nodes.refineNode(nodeId);

  // Set the Address code of the new Node
  auto parentLevel = _nodes.getNodeLevel(nodeId);
  size_t indexStep = 1u << (getMaxLevel() - (parentLevel + 1));
  Eigen::Array3i parentAddressCode = _nodes.getNodeAddressCode(nodeId);
  _nodes.setNodeAddressCode(
      childrenIds[0], Eigen::Array3i(parentAddressCode[0], parentAddressCode[1],
                                     parentAddressCode[2]));
  _nodes.setNodeAddressCode(
      childrenIds[1],
      Eigen::Array3i(parentAddressCode[0] + indexStep, parentAddressCode[1],
                     parentAddressCode[2]));
  _nodes.setNodeAddressCode(
      childrenIds[2],
      Eigen::Array3i(parentAddressCode[0], parentAddressCode[1] + indexStep,
                     parentAddressCode[2]));
  _nodes.setNodeAddressCode(
      childrenIds[3],
      Eigen::Array3i(parentAddressCode[0] + indexStep,
                     parentAddressCode[1] + indexStep, parentAddressCode[2]));
  _nodes.setNodeAddressCode(
      childrenIds[4], Eigen::Array3i(parentAddressCode[0], parentAddressCode[1],
                                     parentAddressCode[2] + indexStep));
  _nodes.setNodeAddressCode(
      childrenIds[5],
      Eigen::Array3i(parentAddressCode[0] + indexStep, parentAddressCode[1],
                     parentAddressCode[2] + indexStep));
  _nodes.setNodeAddressCode(
      childrenIds[6],
      Eigen::Array3i(parentAddressCode[0], parentAddressCode[1] + indexStep,
                     parentAddressCode[2] + indexStep));
  _nodes.setNodeAddressCode(childrenIds[7],
                            Eigen::Array3i(parentAddressCode[0] + indexStep,
                                           parentAddressCode[1] + indexStep,
                                           parentAddressCode[2] + indexStep));

  // Interconnect children
  connectNodes(childrenIds[0], childrenIds[1]);
  connectNodes(childrenIds[0], childrenIds[2]);
  connectNodes(childrenIds[1], childrenIds[3]);
  connectNodes(childrenIds[2], childrenIds[3]);
  connectNodes(childrenIds[4], childrenIds[5]);
  connectNodes(childrenIds[4], childrenIds[6]);
  connectNodes(childrenIds[5], childrenIds[7]);
  connectNodes(childrenIds[6], childrenIds[7]);
  connectNodes(childrenIds[0], childrenIds[4]);
  connectNodes(childrenIds[1], childrenIds[5]);
  connectNodes(childrenIds[2], childrenIds[6]);
  connectNodes(childrenIds[3], childrenIds[7]);

  // TODO: Connect new nodes to the boundary cell, so the boundary will be well
  // defined

  connectChildrenToParentNeighbors(childrenIds, neighbors, parentRegion);

  destroyNode(nodeId);

  return childrenIds;
}

}  // namespace Ramuh
