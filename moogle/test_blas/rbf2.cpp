#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <interpolation/rbf_interpolation.hpp>
#include <exceptions/exception.hpp>

TEST_CASE("RBF Interpolation 2D", "[rbf, interpolation]") {
  Odin::RBFInterpolation2 rbf;

  try {
    rbf.setPolynomialDegreee(1);
  } catch (Bahamut::BahamutException& e) {
    std::cerr << e.what() << '\n';
    if (e.isLethal())
      exit(e.getErrorNumber());
  }
}