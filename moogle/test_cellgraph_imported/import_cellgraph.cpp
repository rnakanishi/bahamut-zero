#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_CONSOLE_WIDTH 300
#include <omp.h>

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
#include <utils/exception.hpp>
#include <vector>

#include "cellgraph_temp.hpp"

double analyticFunction(Eigen::Array2d point) {
  double x = point.x();
  double y = point.y();
  return sin(3 * x) * cos(4 * y);
}

double laplacianFunction(Eigen::Array2d point) {
  return -25 * analyticFunction(point);
}

double laplacianRBF(std::string filename) {
  CellGraphTemp grid(filename);
  std::map<int, double> analyticValues;
  grid.initializeLevelSet();

  std::vector<Eigen::Triplet<double>> triplets;
  std::map<int, int> mapCellIds;
  int countCells = 0;

  // Function to control which ids are mapped in the matrix system
  std::function<int(int)> mapFunction = [&mapCellIds](int cellId) -> int {
    if (mapCellIds.find(cellId) == mapCellIds.end()) {
#pragma omp critical
      {
        int count = mapCellIds.size();
        mapCellIds[cellId] = count;
      }
    }
    return mapCellIds[cellId];
  };

  std::vector<double>& levelset = grid.getLevelsetField();
  std::map<int, double> bValues;
#pragma omp parallel
  {
    Odin::DifferentialRBF2 rbf;
    std::map<int, double> threadAnalyticValues;
    std::map<int, double> threadBValues;
    std::vector<Eigen::Triplet<double>> threadTriplet;
    threadTriplet.clear();
#pragma omp for
    for (size_t cellId = 0; cellId < grid.getCellCount(); cellId++) {
      // check if cell is negative
      if (!grid.isValid(cellId))
        continue;
      if (levelset[cellId] > 0)
        continue;
      rbf.clearSamples();

      std::vector<size_t> neighborsIdsVector;
      auto centerPosition = grid.getCellPosition(cellId);

      int matrixCellId = mapFunction(cellId);

      threadAnalyticValues[matrixCellId] = analyticFunction(centerPosition);
      threadBValues[matrixCellId] = laplacianFunction(centerPosition);

      // check all neighbors to find which of them are negative levelset
      // For laplacian, function values aren't needed
      rbf.addSamplePoint(centerPosition, 0.0);
      neighborsIdsVector.push_back(matrixCellId);

      std::vector<int> neighborCellIds = grid.getNeighborCellsId(cellId);
      int icount = 0;
      std::vector<int> dirichletIndices;
      for (auto& neighborId : neighborCellIds) {
        if (levelset[neighborId] <= 0) {
          size_t neighborMatrixId = mapFunction(neighborId);
          neighborsIdsVector.push_back(neighborMatrixId);
          rbf.addSamplePoint(grid.getCellPosition(neighborId), 0.0);
        } else {
          int surfaceOffset = 0;
          while (mapCellIds.find(neighborId + surfaceOffset) !=
                 mapCellIds.end())
            surfaceOffset += grid.getCellCount();
          int surfaceId = mapFunction(neighborId + surfaceOffset);

          Eigen::Array2d surfacePosition =
              grid.findSurfacePosition(cellId, neighborId);
          rbf.addSamplePoint(surfacePosition, 0.0);

          neighborsIdsVector.push_back(surfaceId);
          dirichletIndices.push_back(surfaceId);

          threadAnalyticValues[surfaceId] = analyticFunction(surfacePosition);
          threadBValues[surfaceId] = analyticFunction(surfacePosition);
        }
        icount++;
      }

      // solve laplacian weights and assign to triplets
      auto weights = rbf.computeLaplacianWeightsAt(centerPosition);

      for (size_t i = 0; i < neighborsIdsVector.size(); i++) {
        threadTriplet.emplace_back(matrixCellId, neighborsIdsVector[i],
                                   weights[i]);
      }
      for (auto dirichlet : dirichletIndices) {
        threadTriplet.emplace_back(dirichlet, dirichlet, 1);
      }
    }
#pragma omp critical
    {
      triplets.insert(triplets.end(), threadTriplet.begin(),
                      threadTriplet.end());
      analyticValues.insert(threadAnalyticValues.begin(),
                            threadAnalyticValues.end());
      bValues.insert(threadBValues.begin(), threadBValues.end());
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
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double>, Eigen::IncompleteLUT<double>>
      solver;
  solver.compute(M);
  x = solver.solve(b);

  Eigen::VectorXd analyticX(x.size());
  for (auto ax : analyticValues) {
    analyticX[ax.first] = ax.second;
  }

  return (x - analyticX).cwiseAbs().maxCoeff();
}

TEST_CASE("Import Cell Graph", "[cellgraph]") {
  // Read cellgraph from furoo library
  std::string filename = "resources/cellgraph.io";
  CellGraphTemp cellgraph(filename);
  cellgraph.statusLog();

  // Assemble cells with their poision and their neighbors
}

TEST_CASE("Laplacian Cell Graph", "[cellgraph, laplacian]") {
  // Read cellgraph from furoo library
  for (int i = 0; i < 9; i++) {
    char filename[256];
    sprintf(filename, "resources/cellgraph.%03d.io", i);
    std::cout << laplacianRBF(filename) << std::endl;
  }

  // Assemble cells with their poision and their neighbors
}