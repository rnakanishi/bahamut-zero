#include <exceptions/exception.hpp>
#include <grid/cell_graph3.hpp>

namespace Ramuh {

CellGraph3::Nodes::Nodes(BoundingBox3 domain) {
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

void CellGraph3::Nodes::destroyNode(size_t nodeId) {
  _validCells[nodeId] = false;
  _availableIds.push(nodeId);
  _cellsEdges[nodeId].clear();
}

int CellGraph3::Nodes::getNodeLevel(size_t nodeId) {
  return _cellsLevel[nodeId];
}

int CellGraph3::Nodes::getCellsCount() {
  return _validCells.size();
}

int CellGraph3::Nodes::getTotalCells() {
  return getCellsCount();
}

Eigen::Array3i CellGraph3::Nodes::getNodeAddressCode(size_t nodeId) {
  return _cellsAddressCode[nodeId];
}

void CellGraph3::Nodes::setNodeAddressCode(size_t nodeId, Eigen::Array3i code) {
  _cellsAddressCode[nodeId] = code;
}

int CellGraph3::Nodes::getValidCellsCount() {
  return getCellsCount() - _availableIds.size();
}

Eigen::Array3d CellGraph3::Nodes::getNodeCenterPosition(size_t nodeId) {
  return _cellsRegion[nodeId].getCenter();
}

BoundingBox3 CellGraph3::Nodes::getNodeRegion(size_t nodeId) {
  return _cellsRegion[nodeId];
}

size_t CellGraph3::Nodes::createNode(BoundingBox3 region, int level) {
  size_t newId;
  if (!_availableIds.empty()) {
    newId = _availableIds.front();
    _availableIds.pop();
  } else {
    newId = _validCells.size();
    _validCells.push_back(true);
    _cellsRegion.push_back(region);
    _cellsAddressCode.emplace_back(0, 0, 0);
    _cellsLevel.push_back(level);
    _cellsEdges.emplace_back(std::list<size_t>());

    // Increase all fields size
    for (auto&& field : _scalarFields) {
      field.emplace_back(0.0);
    }
    for (auto&& field : _vectorFields) {
      field.emplace_back(0.0, 0.0, 0.0);
    }
  }

  _cellsRegion[newId] = region;
  _cellsLevel[newId] = level;
  _validCells[newId] = true;

  return newId;
}

bool CellGraph3::Nodes::areNeighbors(size_t nodeA, size_t nodeB) {
  // TODO: implement CellGraph3::Nodes::areNeighbors
}

size_t CellGraph3::Nodes::addNode(BoundingBox3 region, int level) {
  return createNode(region, level);
}

std::list<size_t>& CellGraph3::Nodes::getNodeEdges(size_t nodeId) {
  return _cellsEdges[nodeId];
}

void CellGraph3::Nodes::addEdge(size_t nodeId, size_t edge) {
  _cellsEdges[nodeId].push_back(edge);
}

void CellGraph3::Nodes::addEdges(size_t nodeId, std::list<size_t> edges) {
  _cellsEdges[nodeId].insert(_cellsEdges[nodeId].end(), edges.begin(),
                             edges.end());
}

void CellGraph3::Nodes::removeEdge(size_t nodeId, size_t edgeId) {
  std::list<size_t>::iterator it = _cellsEdges[nodeId].begin();
  while (it != _cellsEdges[nodeId].end()) {
    if (*it == edgeId) {
      _cellsEdges[nodeId].erase(it++);
      break;
    }
    ++it;
  }
}

std::vector<size_t> CellGraph3::Nodes::refineNode(size_t nodeId) {
  if (nodeId >= getCellsCount() || nodeId <= 0) {
    // Node 0 should never be refined
    Bahamut::UnexpectedParameterException exception(
        306, "CellGraph3::Nodes::refineNode");
    if (nodeId <= 0)
      exception.appendMessage("The 0 index node should never be refined.");
    else
      exception.appendMessage("NodeId bigger than total number of cells");
    throw exception;
  }
  if (!_validCells[nodeId]) {
    Bahamut::UnexpectedParameterException exception(
        306, "CellGraph3::Nodes::refineNode");
    exception.appendMessage("NodeId is not valid.");
    throw exception;
  }

  // find the neighbors, so we can connect them later
  BoundingBox3 parentRegion = _cellsRegion[nodeId];
  int parentLevel = _cellsLevel[nodeId];

  // Create four new nodes with one level higher and set their region
  std::vector<BoundingBox3> childrenRegions = parentRegion.subdivide();
  std::vector<size_t> childrenIds;
  for (size_t i = 0; i < childrenRegions.size(); i++) {
    childrenIds.push_back(createNode(childrenRegions[i], parentLevel + 1));
  }

  return childrenIds;
}

std::vector<size_t> CellGraph3::Nodes::findNodeSiblings(size_t nodeId) {}

}  // namespace Ramuh
