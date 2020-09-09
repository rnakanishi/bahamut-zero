#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <geometry/bounding_box.hpp>
#include <utils/exception.hpp>

TEST_CASE("Bounding box constructors", "[bbox, geometry]") {
  Eigen::Array2d zero(0., 0.);

  SECTION("Default constructor") {
    Ramuh::BoundingBox2 bbox;
    REQUIRE((bbox.getMin() - Eigen::Array2d(-1, -1)).isApprox(zero));
    REQUIRE((bbox.getMax() - Eigen::Array2d(1, 1)).isApprox(zero));
  }

  SECTION("Min and Max constructor") {
    Eigen::Array2d min(-1.5, -2), max(2.75, 4);
    Ramuh::BoundingBox2 bbox(min, max);
    REQUIRE((bbox.getMin() - min).isApprox(zero));
    REQUIRE((bbox.getMax() - max).isApprox(zero));
  }

  SECTION("Double constructor") {
    double min = -1.5, max = 3.6;
    Ramuh::BoundingBox2 bbox(min, max);
    REQUIRE((bbox.getMin() - Eigen::Array2d(min, min)).isApprox(zero));
    REQUIRE((bbox.getMax() - Eigen::Array2d(max, max)).isApprox(zero));
  }
}

TEST_CASE("Bounding box setters and getters", "[bbox, geometry]") {
  Eigen::Array2d zero(0., 0.);
  Ramuh::BoundingBox2 bbox;

  SECTION("Setting double") {
    Eigen::Array2d min(1.5), max(4.0);
    bbox.setMin(1.5);

    REQUIRE((bbox.getMin() - min).isApprox(zero));
    REQUIRE((bbox.getMax() - Eigen::Array2d(1.)).isApprox(zero));

    bbox.setMax(4.0);

    REQUIRE((bbox.getMin() - min).isApprox(zero));
    REQUIRE((bbox.getMax() - max).isApprox(zero));
  }
}

TEST_CASE("Subdivisions", "[bbox, geometry]") {
  Ramuh::BoundingBox2 bbox;

  auto singleDivision = bbox.subdivide();
  auto doubleDivision = bbox.subdivide(3);
  auto xyDivisions = bbox.subdivide(2, 4);
  SECTION("Division count") {
    REQUIRE(singleDivision.size() == 4);
    REQUIRE(doubleDivision.size() == 16);
    REQUIRE(xyDivisions.size() == 15);
  }
  for (auto&& division : singleDivision) {
    std::cerr << division << std::endl;
  }
}