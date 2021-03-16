#include <grid/cell_graph3.hpp>

namespace Ramuh {

CellGraph3::Edges::Edges() {}

std::vector<size_t> CellGraph3::Edges::getEdgeNodes(size_t edgeId) {
  return _edgeNodes[edgeId];
}

size_t CellGraph3::Edges::createEdge(size_t nodeA, size_t nodeB) {
  size_t newId;
  if (!_availableIds.empty()) {
    newId = _availableIds.front();
    _availableIds.pop();

    _edgeNodes[newId] = {nodeA, nodeB};
    _validEdges[newId] = true;
  } else {
    newId = _edgeNodes.size();

    _validEdges.push_back(true);
    _edgeNodes.push_back(std::vector<size_t>());
    _edgeNodes[newId].emplace_back(nodeA);
    _edgeNodes[newId].emplace_back(nodeB);

    for (auto&& field : _scalarFields) {
      field.emplace_back(0.0);
    }
    for (auto&& field : _vectorFields) {
      field.emplace_back(0.0, 0.0, 0.0);
    }
  }

  return newId;
}

void CellGraph3::Edges::destroyEdge(size_t edgeId) {
  _validEdges[edgeId] = false;
  _availableIds.push(edgeId);
}

bool CellGraph3::Edges::isValid(size_t edgeId) {
  return _validEdges[edgeId];
}
}  // namespace Ramuh