#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <interpolation/rbf_interpolation.hpp>
#include <utils/exception.hpp>

TEST_CASE("RBF Interpolation 2D", "[rbf, interpolation]") {
  Giratina::RBFInterpolation2 rbf;

  try {
    rbf.setPolynomialDegreee(1);
  } catch (Arceus::ArceusException &e) {
    std::cerr << e.what() << '\n';
    if (e.isLethal())
      exit(e.getErrorNumber());
  }
}