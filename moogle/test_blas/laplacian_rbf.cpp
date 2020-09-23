#define CATCH_CONFIG_MAIN
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <blas/rbf_finite_difference.hpp>
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

double laplacianRBF(int gridSize) {
  Ramuh::Levelset2 grid(Ramuh::BoundingBox2(0, 1), Eigen::Array2i(gridSize));
  Odin::DifferentialRBF2 rbf;
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
  for (size_t cellId = 0; cellId < grid.getCellCount(); cellId++) {
    rbf.clearSamples();
    // check if cell is negative
    if (levelset[cellId] > 0)
      continue;

    std::vector<size_t> neighborsIdsVector;
    auto centerPosition = grid.getCellPosition(cellId);
    auto h = grid.getH();
    double h2 = h[0] * h[1];
    int matrixCellId = mapFunction(cellId);

    analyticValues[matrixCellId] = analyticFunction(centerPosition);
    bValues[matrixCellId] = laplacianFunction(centerPosition);

    // check all neighbors to find which of them are negative levelset
    // For laplacian, function values aren't needed
    rbf.addSamplePoint(centerPosition, 0.0);
    neighborsIdsVector.push_back(matrixCellId);

    std::vector<size_t> neighborCellIds = grid.getNeighborCellsId(cellId);
    int icount = 0;
    std::vector<int> dirichletIndices;
    for (auto& neighborId : neighborCellIds) {
      if (levelset[neighborId] <= 0) {
        size_t neighborMatrixId = mapFunction(neighborId);
        neighborsIdsVector.push_back(neighborMatrixId);
        rbf.addSamplePoint(grid.getCellPosition(neighborId), 0.0);
      } else {
        // Find surface direction
        Ramuh::Levelset2::Direction surfaceDirection;
        if (std::abs((int)cellId - (int)neighborId) == 1)
          surfaceDirection = Ramuh::Levelset2::Direction::HORIZONTAL;
        else
          surfaceDirection = Ramuh::Levelset2::Direction::VERTICAL;

        int surfaceOffset = 0;
        while (mapCellIds.find(neighborId + surfaceOffset) != mapCellIds.end())
          surfaceOffset += grid.getCellCount();
        int surfaceId = mapFunction(neighborId + surfaceOffset);

        Eigen::Array2d surfacePosition =
            grid.findSurfacePosition(cellId, surfaceDirection);
        rbf.addSamplePoint(surfacePosition, 0.0);

        neighborsIdsVector.push_back(surfaceId);
        dirichletIndices.push_back(surfaceId);

        analyticValues[surfaceId] = bValues[surfaceId] =
            analyticFunction(surfacePosition);
        // bValues[neighborMatrixId] = analyticFunction(surfacePosition);
      }
      icount++;
    }

    // solve laplacian weights and assign to triplets
    auto weights = rbf.computeLaplacianWeightsAt(centerPosition);

    for (size_t i = 0; i < neighborsIdsVector.size(); i++) {
      triplets.emplace_back(matrixCellId, neighborsIdsVector[i], weights[i]);
    }
    for (auto dirichlet : dirichletIndices) {
      triplets.emplace_back(dirichlet, dirichlet, 1);
    }
  }

  // Assembly poisson system solver
  Eigen::SparseMatrix<double> M(mapCellIds.size(), mapCellIds.size());
  Eigen::VectorXd b, x;
  b = Eigen::VectorXd::Zero(mapCellIds.size());

  M.setFromTriplets(triplets.begin(), triplets.end());
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

  return (x - analyticX).cwiseAbs().maxCoeff();
}

TEST_CASE("Laplacian RBF", "[finite_difference, levelset]") {
  std::vector<int> gridSizes = {4, 8, 16, 32, 64, 128, 256};
  std::vector<double> errors;

  for (int gridSize : gridSizes) {
    std::cout << "Computing: " << gridSize << std::endl;
    errors.emplace_back(laplacianRBF(gridSize));
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