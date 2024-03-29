#include <grid/cell_centered_grid.hpp>
#include <exceptions/exception.hpp>

namespace Ramuh {

CellCenteredGrid2::CellCenteredGrid2()
    : CellCenteredGrid2(BoundingBox2(), Eigen::Array2i(32, 32)) {}

CellCenteredGrid2::CellCenteredGrid2(BoundingBox2 domain,
                                     Eigen::Array2i gridSize)
    : _domain(domain), _gridSize(gridSize) {}

void CellCenteredGrid2::setGridSize(Eigen::Array2i size) {
  _gridSize = size;
}

void CellCenteredGrid2::setGridSize(int sizeX, int sizeY) {
  setGridSize(Eigen::Array2i(sizeX, sizeY));
}

void CellCenteredGrid2::setDomain(BoundingBox2 domain) {
  _domain = domain;
}

void CellCenteredGrid2::setDomain(Eigen::Array2d min, Eigen::Array2d max) {
  setDomain(BoundingBox2(min, max));
}

size_t CellCenteredGrid2::getId(size_t i, size_t j) {
  return j * _gridSize[0] + i;
}

std::vector<size_t> CellCenteredGrid2::getIj(size_t id) {
  std::vector<size_t> index(2);
  index[0] = id % (_gridSize[0]);
  index[1] = id / _gridSize[0];
  return index;
}

std::vector<size_t> CellCenteredGrid2::getNeighborCellsId(size_t cellId) {
  if (cellId > getCellCount() || cellId < 0)
    throw(Bahamut::UnexpectedParameterException(
        306, "Levelset2::isSurface(cellId)"));
  std::vector<size_t> neighborsId;
  auto ij = getIj(cellId);
  auto resolution = getResolution();
  if (ij[0] >= 0)
    neighborsId.push_back(getId(ij[0] - 1, ij[1]));
  if (ij[0] < resolution[0] - 1)
    neighborsId.push_back(getId(ij[0] + 1, ij[1]));
  if (ij[1] >= 0)
    neighborsId.push_back(getId(ij[0], ij[1] - 1));
  if (ij[1] < resolution[1] - 1)
    neighborsId.push_back(getId(ij[0], ij[1] + 1));
  return neighborsId;
}

Eigen::Array2i CellCenteredGrid2::getGridSize() {
  return _gridSize;
}

Eigen::Array2i CellCenteredGrid2::getResolution() {
  return _gridSize;
}

BoundingBox2 CellCenteredGrid2::getDomain() {
  return _domain;
}

Eigen::Array2d CellCenteredGrid2::getH() {
  return _domain.getSize().cwiseQuotient(_gridSize.cast<double>());
}

Eigen::Array2d CellCenteredGrid2::getSpacing() {
  return getH();
}

Eigen::Array2d CellCenteredGrid2::getCellPosition(int i, int j) {
  auto h = getH();
  return _domain.getMin() + Eigen::Array2d((i + 0.5) * h[0], (j + 0.5) * h[1]);
}

Eigen::Array2d CellCenteredGrid2::getCellPosition(int id) {
  auto ij = getIj(id);
  auto h = getH();
  int i = ij[0], j = ij[1];
  return _domain.getMin() + Eigen::Array2d((i + 0.5) * h[0], (j + 0.5) * h[1]);
}

BoundingBox2 CellCenteredGrid2::getCellBoundingBox(int i, int j) {
  auto position = getCellPosition(i, j);
  BoundingBox2 box;
  Eigen::Array2d h = getH();
  box.setMin(position - h / 2);
  box.setMax(position + h / 2);
  return box;
}

BoundingBox2 CellCenteredGrid2::getCellBoundingBox(int id) {
  auto ij = getIj(id);
  return getCellBoundingBox(ij[0], ij[1]);
}

int CellCenteredGrid2::getCellCount() {
  return _gridSize.prod();
}

size_t CellCenteredGrid2::createCellScalarField(std::string label) {
  return createCellScalarField(label, 0);
}

size_t CellCenteredGrid2::createCellScalarField(std::string label,
                                                double value) {
  if (_cellScalarLabels.find(label) == _cellScalarLabels.end()) {
    _cellScalarFields.emplace_back(
        std::vector<double>(_gridSize.prod(), value));
    _cellScalarLabels[label] = _cellScalarFields.size() - 1;
  }
  return _cellScalarLabels[label];
}

std::vector<double>& CellCenteredGrid2::getCellScalarField(std::string label) {
  return _cellScalarFields[_cellScalarLabels[label]];
}

std::vector<double>& CellCenteredGrid2::getCellScalarField(size_t id) {
  return _cellScalarFields[id];
}

size_t CellCenteredGrid2::createCellVectorField(std::string label) {
  return createCellVectorField(label, Eigen::Vector2d(0., 0.));
}

size_t CellCenteredGrid2::createCellVectorField(std::string label,
                                                Eigen::Vector2d value) {
  if (_cellVectorLabels.find(label) == _cellVectorLabels.end()) {
    _cellVectorFields.emplace_back(
        std::vector<Eigen::Vector2d>(_gridSize.prod(), value));
    _cellVectorLabels[label] = _cellVectorFields.size() - 1;
  }
  return _cellVectorLabels[label];
}

std::vector<Eigen::Vector2d>& CellCenteredGrid2::getCellVectorField(
    std::string label) {
  return _cellVectorFields[_cellVectorLabels[label]];
}

std::vector<Eigen::Vector2d>& CellCenteredGrid2::getCellVectorField(int id) {
  return _cellVectorFields[id];
}

double CellCenteredGrid2::interpolateCellScalarField(std::string label,
                                                     Eigen::Array2d position) {
  return interpolateCellScalarField(_cellScalarLabels[label], position);
}

double CellCenteredGrid2::interpolateCellScalarField(int dataId,
                                                     Eigen::Array2d position) {
  auto& data = getCellScalarField(dataId);
  auto h = getH();

  // Find which cell-id data belongs
  Eigen::Array2i cellId =
      (position - _domain.getMin()).cwiseQuotient(h).floor().cast<int>();

  // Assemble bilinear stencil interpolation for velocities
  auto cellPos = getCellPosition(cellId[0], cellId[1]);

  int xindex, yindex;
  xindex = std::min(_gridSize[0] - 1, cellId[0] + 1);
  yindex = std::min(_gridSize[1] - 1, cellId[1] + 1);
  if (position[0] < cellPos[0]) {
    xindex = std::max(0, cellId[0] - 1);
    h[0] = -h[0];
  }
  if (position[1] < cellPos[1]) {
    yindex = std::max(0, cellId[1] - 1);
    h[1] = -h[1];
  }

  std::vector<Eigen::Array2d> points;
  std::vector<double> values(4);
  double target[2];
  target[0] = position[0];
  target[1] = position[1];

  // cell Stencil for linear interpolation
  points.emplace_back(cellPos);
  points.emplace_back(cellPos + Eigen::Array2d(h[0], 0));
  points.emplace_back(cellPos + Eigen::Array2d(0, h[1]));
  points.emplace_back(cellPos + h);

  values[0] = data[getId(cellId[0], cellId[1])];
  values[1] = data[getId(xindex, cellId[1])];
  values[2] = data[getId(cellId[0], yindex)];
  values[3] = data[getId(xindex, yindex)];

  // return Ramuh::Interpolator::bilinear(target, points, values);
  throw(Bahamut::BahamutException(
      102, "CellCenteredGrid2::interpolateCellScalarField"));
  return 0.0;
}

Eigen::Vector2d CellCenteredGrid2::interpolateCellVectorField(
    std::string label,
    Eigen::Array2d position) {
  return interpolateCellVectorField(_cellVectorLabels[label], position);
}

Eigen::Vector2d CellCenteredGrid2::interpolateCellVectorField(
    int dataId,
    Eigen::Array2d position) {
  auto& data = getCellVectorField(dataId);
  auto h = getH();

  // Find which cell-id data belongs
  Eigen::Array2i cellId =
      (position - _domain.getMin()).cwiseQuotient(h).floor().cast<int>();

  // Assemble bilinear stencil interpolation for velocities
  auto cellPos = getCellPosition(cellId[0], cellId[1]);

  int xindex, yindex;
  xindex = std::min(_gridSize[0] - 1, cellId[0] + 1);
  yindex = std::min(_gridSize[1] - 1, cellId[1] + 1);
  if (position[0] < cellPos[0]) {
    xindex = std::max(0, cellId[0] - 1);
    h[0] = -h[0];
  }
  if (position[1] < cellPos[1]) {
    yindex = std::max(0, cellId[1] - 1);
    h[1] = -h[1];
  }

  std::vector<Eigen::Array2d> points;
  std::vector<double> values(4);
  double target[2];
  target[0] = position[0];
  target[1] = position[1];

  Eigen::Array2d interpData;
  // cell Stencil for linear interpolation
  points.emplace_back(cellPos);
  points.emplace_back(cellPos + Eigen::Array2d(h[0], 0));
  points.emplace_back(cellPos + Eigen::Array2d(0, h[1]));
  points.emplace_back(cellPos + h);

  for (size_t d = 0; d < 2; d++) {
    values[0] = data[getId(cellId[0], cellId[1])][d];
    values[1] = data[getId(xindex, cellId[1])][d];
    values[2] = data[getId(cellId[0], yindex)][d];
    values[3] = data[getId(xindex, yindex)][d];
    // interpData[d] = Ramuh::Interpolator::bilinear(target, points, values);
  }

  throw(Bahamut::BahamutException(
      102, "CellCenteredGrid2::interpolateCellVectorField"));
  return interpData;
}

}  // namespace Ramuh