
#include "cellgraph_temp.hpp"

CellGraphTemp::CellGraphTemp(std::string filename) {
  std::fstream fin(filename, std::fstream::in);
  if (!fin.is_open()) {
    std::cerr << "Error reading the file\n";
    exit(-1);
  }

  {
    int nVertices;
    fin >> nVertices;
    vertices.resize(nVertices);
    for (size_t i = 0; i < nVertices; i++) {
      int vid;
      Eigen::Array2d position;
      fin >> vid >> position[0] >> position[1];
      vertices[i] = position;
      fin.ignore(100, '\n');
    }
  }
  {
    int nCells;
    fin >> nCells;
    cellCount = nCells;
    for (size_t i = 0; i < nCells; i++) {
      int cellId;
      Eigen::Array2d position(0.0);
      fin >> cellId;
      if (cellId >= cellCount)
        cellCount = cellId + 1;
      for (size_t v = 0; v < 4; v++) {
        int vid;
        fin >> vid;
        position += vertices[vid];
      }
      cellCenters[cellId] = position / 4.;
      fin.ignore(100, '\n');
    }
  }
  {
    int nEdges;
    fin >> nEdges;
    for (size_t i = 0; i < nEdges; i++) {
      int edgeId, side, cellA, cellB;
      fin >> edgeId >> side >> cellA >> cellB;
      if (!(cellA == 0 || cellB == 0)) {
        connections[cellA].push_back(cellB);
        connections[cellB].push_back(cellA);
      }
      fin.ignore(100, '\n');
    }
  }
}

void CellGraphTemp::statusLog() {
  std::cout << "Vertices: " << vertices.size() << std::endl;
  for (size_t i = 0; i < vertices.size(); i++) {
    std::cout << vertices[i].transpose() << std::endl;
  }
  std::cout << "\nCells: " << cellCenters.size() << std::endl;
  for (auto cell : cellCenters) {
    std::cout << cell.second.transpose() << std::endl;
  }
  std::cout << "\nEdges: " << connections.size() << std::endl;
  for (auto edge : connections) {
    std::cout << edge.first << ": ";
    for (auto cell : edge.second) {
      std::cout << cell << " ";
    }
    std::cout << std::endl;
  }
}

int CellGraphTemp::getCellCount() { return cellCount; }

bool CellGraphTemp::isValid(int cellId) {
  if (cellCenters.find(cellId) == cellCenters.end())
    return false;
  return true;
}

Eigen::Array2d CellGraphTemp::getCellPosition(int cellId) {
  return cellCenters[cellId];
}

std::vector<int> &CellGraphTemp::getNeighborCellsId(int cellId) {
  return connections[cellId];
}

std::vector<double> &CellGraphTemp::getLevelsetField() { return levelsetField; }

void CellGraphTemp::initializeLevelSet() {
  levelsetField.resize(cellCount);
  for (auto cell : cellCenters) {
    double x = cell.second[0], y = cell.second[1];
    x -= 1;
    y -= 1;
    levelsetField[cell.first] = x * x + y * y - 0.25;
  }
}

Eigen::Array2d CellGraphTemp::findSurfacePosition(int cellId, int neighborId) {
  Eigen::Array2d cellPosition = getCellPosition(cellId);
  Eigen::Array2d neighborCellPosition = getCellPosition(neighborId);
  double distance, theta;

  for (size_t d = 0; d < 2; d++) {
    double cellD, neighborD;
    cellD = cellPosition[d];
    neighborD = neighborCellPosition[d];
    distance = cellD - neighborD;
    theta = -levelsetField[cellId] /
            (levelsetField[neighborId] - levelsetField[cellId]);
    cellPosition[d] += theta * distance;
  }

  return cellPosition;
}
