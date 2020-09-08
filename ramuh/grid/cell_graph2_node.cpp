#include <grid/cell_graph2.hpp>
#include <utils/exception.hpp>

namespace Palkia {

CellGraph2::Nodes::Nodes(BoundingBox2 domain) {
  _cellsRegion.push_back(domain);
  _cellsRegion.push_back(domain);

  _cellsAddressCode.emplace_back(0, 0);
  _cellsAddressCode.emplace_back(0, 0);

  _cellsLevel.push_back(1);
  _cellsLevel.push_back(1);

  _validCells.push_back(true);
  _validCells.push_back(true);

  _boundaryCells.push_back(true);
  _boundaryCells.push_back(true);

  _cellsEdges.push_back(std::list<size_t>());
  _cellsEdges.push_back(std::list<size_t>());
}

void CellGraph2::Nodes::destroyNode(size_t nodeId) {
  _validCells[nodeId] = false;
  _availableIds.push(nodeId);
  _cellsEdges[nodeId].clear();
}

int CellGraph2::Nodes::getNodeLevel(size_t nodeId) {
  return _cellsLevel[nodeId];
}

int CellGraph2::Nodes::getCellsCount() {
  return _validCells.size();
}

int CellGraph2::Nodes::getTotalCells() {
  return getCellsCount();
}

Eigen::Array2i CellGraph2::Nodes::getNodeAddressCode(size_t nodeId) {
  return _cellsAddressCode[nodeId];
}

void CellGraph2::Nodes::setNodeAddressCode(size_t nodeId, Eigen::Array2i code) {
  _cellsAddressCode[nodeId] = code;
}

int CellGraph2::Nodes::getValidCellsCount() {
  return getCellsCount() - _availableIds.size();
}

Eigen::Array2d CellGraph2::Nodes::getNodeCenterPosition(size_t nodeId) {
  return _cellsRegion[nodeId].getCenter();
}

BoundingBox2 CellGraph2::Nodes::getNodeRegion(size_t nodeId) {
  return _cellsRegion[nodeId];
}

size_t CellGraph2::Nodes::createNode(BoundingBox2 region, int level) {
  size_t newId;
  if (!_availableIds.empty()) {
    newId = _availableIds.front();
    _availableIds.pop();
  } else {
    newId = _validCells.size();
    _validCells.push_back(true);
    _cellsRegion.push_back(region);
    _cellsAddressCode.emplace_back(0, 0);
    _cellsLevel.push_back(level);
    _cellsEdges.emplace_back(std::list<size_t>());

    // Increase all fields size
    for (auto&& field : _scalarFields) {
      field.emplace_back(0.0);
    }
    for (auto&& field : _vectorFields) {
      field.emplace_back(0.0, 0.0);
    }
  }

  _cellsRegion[newId] = region;
  _cellsLevel[newId] = level;
  _validCells[newId] = true;

  return newId;
}

bool CellGraph2::Nodes::areNeighbors(size_t nodeA, size_t nodeB) {
  // TODO: implement CellGraph2::Nodes::areNeighbors
}

size_t CellGraph2::Nodes::addNode(BoundingBox2 region, int level) {
  return createNode(region, level);
}

std::list<size_t>& CellGraph2::Nodes::getNodeEdges(size_t nodeId) {
  return _cellsEdges[nodeId];
}

void CellGraph2::Nodes::addEdge(size_t nodeId, size_t edge) {
  _cellsEdges[nodeId].push_back(edge);
}

void CellGraph2::Nodes::addEdges(size_t nodeId, std::list<size_t> edges) {
  _cellsEdges[nodeId].insert(_cellsEdges[nodeId].end(), edges.begin(),
                             edges.end());
}

void CellGraph2::Nodes::removeEdge(size_t nodeId, size_t edgeId) {
  std::list<size_t>::iterator it = _cellsEdges[nodeId].begin();
  while (it != _cellsEdges[nodeId].end()) {
    if (*it == edgeId) {
      _cellsEdges[nodeId].erase(it++);
      break;
    }
    ++it;
  }
}

std::vector<size_t> CellGraph2::Nodes::refineNode(size_t nodeId) {
  if (nodeId >= getCellsCount() || nodeId <= 0) {
    // Node 0 should never be refined
    Arceus::UnexpectedParameterException exception(
        306, "CellGraph2::Nodes::refineNode");
    if (nodeId <= 0)
      exception.appendMessage("The 0 index node should never be refined.");
    else
      exception.appendMessage("NodeId bigger than total number of cells");
    throw exception;
  }
  if (!_validCells[nodeId]) {
    Arceus::UnexpectedParameterException exception(
        306, "CellGraph2::Nodes::refineNode");
    exception.appendMessage("NodeId is not valid.");
    throw exception;
  }

  // find the neighbors, so we can connect them later
  BoundingBox2 parentRegion = _cellsRegion[nodeId];
  int parentLevel = _cellsLevel[nodeId];

  // Create four new nodes with one level higher and set their region
  std::vector<BoundingBox2> childrenRegions = parentRegion.subdivide();
  std::vector<size_t> childrenIds;
  for (size_t i = 0; i < 4; i++) {
    childrenIds.push_back(createNode(childrenRegions[i], parentLevel + 1));
  }

  return childrenIds;
}

std::vector<size_t> CellGraph2::Nodes::findNodeSiblings(size_t nodeId) {}

}  // namespace Palkia
