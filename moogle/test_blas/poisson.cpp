#define CATCH_CONFIG_MAIN
#include <blas/rbf_finite_difference.hpp>
#include <catch2/catch.hpp>
#include <grid/cell_centered_grid.hpp>

// Function that will be used to evaluate points over the domain
double function(Eigen::Array2d point) {
  double x = point.x();
  double y = point.y();

  return x + y;
}

TEST_CASE("Poisson for regular grid", "[poisson, rbf]") {
  Odin::DifferentialRBF2 rbf;
  Ramuh::CellCenteredGrid2 domain;

  // Populate domain with levelset

  // For negative values of levelset, add them to the evaluation nodes
}
