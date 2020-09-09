#include <omp.h>

#include <geometry/levelset.hpp>
#include <utils/exception.hpp>

namespace Ramuh {
Levelset2::Levelset2() : Levelset2(BoundingBox2(), Eigen::Array2i(32, 32)) {}

Levelset2::Levelset2(BoundingBox2 domain, Eigen::Array2i gridSize)
    : CellCenteredGrid2(domain, gridSize) {
  _levelsetFieldId = createCellScalarField("levelset");
}

void Levelset2::initializeLevelset(std::function<double(Eigen::Array2d)> phi) {
  std::vector<double>& levelsetField = getCellScalarField(_levelsetFieldId);

#pragma omp parallel for
  for (size_t cellId = 0; cellId < getCellCount(); cellId++) {
    Eigen::Array2d cellPosition = getCellPosition(cellId);
    levelsetField[cellId] = phi(cellPosition);
  }
}

void Levelset2::initializeLevelset(Levelset2::Shape shape) {
  switch (shape) {
    case Levelset2::Shape::CIRCLE:
      initializeLevelset([&](Eigen::Array2d position) -> double {
        auto center = _domain.getCenter();
        double size = _domain.getSize()[0] / 4;
        double x = position[0] - center.x();
        double y = position[1] - center.y();
        return x * x + y * y - size * size;
      });
      break;
    case Levelset2::Shape::SQUARE:
      throw(Bahamut::UnexpectedParameterException(
          305, "Levelset2::initializeLevelset(Levelset2::Shape)"));
      initializeLevelset([](Eigen::Array2d position) -> double { return 0.0; });
      break;
    default:
      throw(Bahamut::UnexpectedParameterException(
          304, "Levelset2::initializeLevelset(Levelset2::Shape)"));
      break;
  }
}

bool Levelset2::isSurface(size_t cellId) {
  if (cellId > getCellCount() || cellId < 0)
    throw(Bahamut::UnexpectedParameterException(
        306, "Levelset2::isSurface(cellId)"));

  std::vector<double> levelsetField = getCellScalarField(_levelsetFieldId);
  // Return false straight away if the cell is positive
  if (levelsetField[cellId] > 0.)
    return false;

  Eigen::Array2i gridSize = getGridSize();
  std::vector<size_t> ij = getIj(cellId);
  size_t i = ij[0], j = ij[1];

  if (i > 0 && levelsetField[getId(i - 1, j)] >= 0)
    return true;
  if (i < gridSize[0] - 1 && levelsetField[getId(i + 1, j)] >= 0)
    return true;
  if (j > 0 && levelsetField[getId(i, j - 1)] >= 0)
    return true;
  if (j < gridSize[1] - 1 && levelsetField[getId(i, j + 1)] >= 0)
    return true;
  return false;
}

std::vector<Levelset2::Direction> Levelset2::findSurfaceDirection(
    size_t cellId) {
  std::vector<double> levelsetField = getCellScalarField(_levelsetFieldId);
  auto cellPosition = getCellPosition(cellId);
  Eigen::Array2i gridSize = getGridSize();
  auto ij = getIj(cellId);
  size_t i = ij[0], j = ij[1];

  if (levelsetField[cellId] > 0.) {
    throw(Bahamut::ConditionsNotMatchException(
        403, "Levelset2::findSurfaceDirection"));
  }

  std::vector<Levelset2::Direction> allDirections;
  if (i > 0 && levelsetField[getId(i - 1, j)] >= 0)
    allDirections.push_back(Levelset2::Direction::HORIZONTAL);
  if (i < gridSize[0] - 1 && levelsetField[getId(i + 1, j)] >= 0)
    allDirections.push_back(Levelset2::Direction::HORIZONTAL);
  if (j > 0 && levelsetField[getId(i, j - 1)] >= 0)
    allDirections.push_back(Levelset2::Direction::VERTICAL);
  if (j < gridSize[1] - 1 && levelsetField[getId(i, j + 1)] >= 0)
    allDirections.push_back(Levelset2::Direction::VERTICAL);

  return allDirections;
}

Eigen::Array2d Levelset2::findSurfacePosition(size_t cellId) {
  try {
    std::vector<Levelset2::Direction> directions;
    // TODO: Add treatment for cases where more than one direction is found
    findSurfacePosition(cellId, directions[0]);
  } catch (const Bahamut::BahamutException exception) {
    Bahamut::BahamutException newException;
    newException.appendMessage(exception.what());
    newException.setCallingLocation("Levelset2::findSurfaceDirection");
    throw(newException);
  }
}

Eigen::Array2d Levelset2::findSurfacePosition(size_t cellId,
                                              Direction direction) {
  if (!isSurface(cellId)) {
    Bahamut::ConditionsNotMatchException exception(
        403, "Leveset2::findSurfacePosition");
    exception.setLethality(false);
    throw(exception);
  }
  std::vector<double> levelsetField = getCellScalarField(_levelsetFieldId);
  Eigen::Array2d cellPosition = getCellPosition(cellId);
  Eigen::Array2d neighborCellPosition;
  Eigen::Array2d spacing = getSpacing();
  double h, theta;
  size_t neighborId;
  std::vector<size_t> ij;

  switch (direction) {
    case Levelset2::Direction::HORIZONTAL:
      h = spacing[0];
      ij = getIj(cellId);
      if (levelsetField[getId(ij[0] - 1, ij[1])] >= 0) {
        neighborId = getId(ij[0] - 1, ij[1]);
        neighborCellPosition = getCellPosition(getId(ij[0] - 1, ij[1]));
        h *= -1;
      } else if (levelsetField[getId(ij[0] + 1, ij[1])] >= 0) {
        neighborId = getId(ij[0] + 1, ij[1]);
        neighborCellPosition = getCellPosition(getId(ij[0] + 1, ij[1]));
      } else {
        Bahamut::ConditionsNotMatchException exception(
            404, "Leveset2::findSurfacePosition");
        exception.setLethality(false);
        throw(exception);
      }
      theta = -levelsetField[cellId] /
              (levelsetField[neighborId] - levelsetField[cellId]);
      cellPosition[0] += theta * h;
      break;

    // =================================================================
    case Levelset2::Direction::VERTICAL:
      h = spacing[1];
      ij = getIj(cellId);
      if (levelsetField[getId(ij[0], ij[1] - 1)] >= 0) {
        neighborId = getId(ij[0], ij[1] - 1);
        neighborCellPosition = getCellPosition(getId(ij[0], ij[1] - 1));
        h *= -1;
      } else if (levelsetField[getId(ij[0], ij[1] + 1)] >= 0) {
        neighborId = getId(ij[0], ij[1] + 1);
        neighborCellPosition = getCellPosition(getId(ij[0], ij[1] + 1));
      } else {
        Bahamut::ConditionsNotMatchException exception(
            405, "Leveset2::findSurfacePosition");
        exception.setLethality(false);
        throw(exception);
      }
      theta = -levelsetField[cellId] /
              (levelsetField[neighborId] - levelsetField[cellId]);
      cellPosition[1] += theta * h;
      break;
  }
  return cellPosition;
}

std::vector<double>& Levelset2::getLevelsetField() {
  return getCellScalarField(_levelsetFieldId);
}

}  // namespace Ramuh
