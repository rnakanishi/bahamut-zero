#define CATCH_CONFIG_MAIN
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <catch2/catch.hpp>
#include <cmath>
#include <functional>
#include <geometry/levelset.hpp>
#include <iostream>
#include <map>
#include <random>
#include <exceptions/exception.hpp>
#include <vector>

double analyticFunction(Eigen::Array2d point) {
  double x = point.x();
  double y = point.y();
  return sin(3 * x) * cos(4 * y);
}

double laplacianFunction(Eigen::Array2d point) {
  return -25 * analyticFunction(point);
}

double laplacianFD(int gridSize) {
  Ramuh::Levelset2 grid(Ramuh::BoundingBox2(0, 1), Eigen::Array2i(gridSize));
  std::map<int, double> analyticValues;
  grid.initializeLevelset(Ramuh::Levelset2::Shape::CIRCLE);

  std::vector<Eigen::Triplet<double>> triplets;
  std::map<int, int> mapCellIds;
  int countCells = 0;

  // Function to control which ids are mapped in the matrix system
  std::function<int(int)> mapFunction = [&mapCellIds](int cellId) -> int {
    if (mapCellIds.find(cellId) == mapCellIds.end()) {
      int count = mapCellIds.size();
      mapCellIds[cellId] = count;
    }
    return mapCellIds[cellId];
  };

  std::vector<double>& levelset = grid.getLevelsetField();
  std::map<int, double> bValues;
  for (int cellId = 0; cellId < grid.getCellCount(); cellId++) {
    // check if cell is negative
    if (levelset[cellId] > 0)
      continue;

    auto centerPosition = grid.getCellPosition(cellId);
    auto h = grid.getH();
    double h2 = h[0] * h[1];
    int matrixCellId = mapFunction(cellId);
    int surfaceIdsCount = 0;

    analyticValues[matrixCellId] = analyticFunction(centerPosition);
    // check all neighbors to find which of them are negative levelset
    // Those that are not negative, compute the weights using Gibou
    // extrapolation method
    bValues[matrixCellId] = laplacianFunction(centerPosition);

    std::vector<size_t> neighborCellIds = grid.getNeighborCellsId(cellId);

    if (levelset[cellId] > 0) {
      bValues[cellId] = analyticFunction(centerPosition);
      triplets.emplace_back(cellId, cellId, 1);
    } else
      for (auto& neighborId : neighborCellIds) {
        if (levelset[neighborId] <= 0) {
          size_t neighborMatrixId = mapFunction(neighborId);
          triplets.emplace_back(matrixCellId, matrixCellId, -1. / h2);
          triplets.emplace_back(matrixCellId, neighborMatrixId, 1. / h2);
        } else {
          // Find surface direction
          Ramuh::Levelset2::Direction surfaceDirection;
          if (std::abs((int)cellId - (int)neighborId) == 1)
            surfaceDirection = Ramuh::Levelset2::Direction::HORIZONTAL;
          else
            surfaceDirection = Ramuh::Levelset2::Direction::VERTICAL;

          int surfaceOffset = 0;
          while (mapCellIds.find(neighborId + surfaceOffset) !=
                 mapCellIds.end())
            surfaceOffset += grid.getCellCount();
          int surfaceId = mapFunction(neighborId + surfaceOffset);

          // Gibou extrapolation
          Eigen::Array2d surfacePosition =
              grid.findSurfacePosition(cellId, surfaceDirection);
          double theta =
              (surfacePosition - centerPosition).matrix().norm() / h[0];

          triplets.emplace_back(matrixCellId, matrixCellId,
                                (-1.) / (theta * h2));

          triplets.emplace_back(matrixCellId, surfaceId, 1 / (theta * h2));
          triplets.emplace_back(surfaceId, surfaceId, 1);

          analyticValues[surfaceId] = bValues[surfaceId] =
              analyticFunction(surfacePosition);
        }
      }
  }
  // Assembly poisson system solver
  Eigen::SparseMatrix<double> M(mapCellIds.size(), mapCellIds.size());
  Eigen::VectorXd b, x;
  b = Eigen::VectorXd::Zero(mapCellIds.size());

  M.setFromTriplets(triplets.begin(), triplets.end());
  // std::cerr << M << std::endl;
  for (auto bMap : bValues) {
    b[bMap.first] = bMap.second;
  }
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
  solver.compute(M);
  x = solver.solve(b);

  Eigen::VectorXd analyticX(x.size());
  for (auto ax : analyticValues) {
    analyticX[ax.first] = ax.second;
  }

  // for (size_t i = 0; i < x.size(); i++) {
  //   std::cerr << x[i] << " " << analyticX[i] << std::endl;
  // }

  return (x - analyticX).cwiseAbs().maxCoeff();
}

TEST_CASE("Traditional finite differecens", "[finite_difference, levelset]") {
  std::vector<int> gridSizes = {4, 8, 16, 32, 64, 128, 256, 512};
  std::vector<double> errors;

  for (int gridSize : gridSizes) {
    std::cout << "Computing: " << gridSize << std::endl;
    errors.emplace_back(laplacianFD(gridSize));
  }
  std::cout << "Grid size, Error, Convergence\n";
  for (int i = 0; i < gridSizes.size(); i++) {
    std::cout << gridSizes[i] << ", ";
    std::cout << errors[i];

    if (i > 0) {
      double conv = std::log2(errors[i - 1] / errors[i]);
      std::cout << ", " << conv;
    }
    std::cout << std::endl;
  }
}