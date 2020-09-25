#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <exceptions/exception.hpp>
#include <geometry/bounding_box.hpp>

TEST_CASE("Bounding box constructors", "[bbox, geometry]") {
  Eigen::Array2d zero(0., 0.);

  SECTION("Default constructor") {
    Ramuh::BoundingBox2 bbox;
    REQUIRE((bbox.getMin() - Eigen::Array2d(-1, -1)).isApprox(zero));
    REQUIRE((bbox.getMax() - Eigen::Array2d(1, 1)).isApprox(zero));
  }

  SECTION("Min and Max constructor") {
    {
      Eigen::Array2d min(-1.5, -2), max(2.75, 4);
      Ramuh::BoundingBox2 bbox(min, max);
      REQUIRE((bbox.getMin() - min).isApprox(zero));
      REQUIRE((bbox.getMax() - max).isApprox(zero));
    }
    {
      Eigen::Array2d min(-1.5, 2), max(2.75, -4);
      Ramuh::BoundingBox2 bbox(min, max);
      REQUIRE((bbox.getMin()).isApprox(Eigen::Array2d(-1.5, -4)));
      REQUIRE((bbox.getMax()).isApprox(Eigen::Array2d(2.75, 2)));
    }
    {
      Eigen::Array3d min(-1.5, -2, -4.1), max(2.75, 4, 3.7);
      Ramuh::BoundingBox3 bbox(min, max);
      REQUIRE((bbox.getMin() - min).isApprox(Eigen::Array3d(0, 0, 0)));
      REQUIRE((bbox.getMax() - max).isApprox(Eigen::Array3d(0, 0, 0)));
    }
    {
      Eigen::Array3d min(-1.5, 1.8, -4.1), max(2.75, -1, 3.7);
      Ramuh::BoundingBox3 bbox(min, max);
      REQUIRE((bbox.getMin()).isApprox(Eigen::Array3d(-1.5, -1, -4.1)));
      REQUIRE((bbox.getMax()).isApprox(Eigen::Array3d(2.75, 1.8, 3.7)));
    }
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

TEST_CASE("Subdivisions 2D", "[bbox, geometry]") {
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

TEST_CASE("Subdivisions 3D", "[bbox, geometry]") {
  Ramuh::BoundingBox3 bbox;

  auto singleDivision = bbox.subdivide();
  auto doubleDivision = bbox.subdivide(3);
  auto xyzDivisions = bbox.subdivide(2, 4, 3);
  SECTION("Division count") {
    REQUIRE(singleDivision.size() == 8);
    REQUIRE(doubleDivision.size() == 64);
    REQUIRE(xyzDivisions.size() == 60);
  }
  for (auto&& division : singleDivision) {
    std::cerr << division << std::endl;
  }
}
TEST_CASE("Cubify", "[bbox, geometry]") {
  Ramuh::BoundingBox3 bbox(Eigen::Array3d(0, 0, 0),
                           Eigen::Array3d(1.2, 1.4, 1));
  bbox.cubify();
  REQUIRE(bbox.getSize().isApprox(Eigen::Array3d(1.4, 1.4, 1.4)));
  REQUIRE(bbox.getMin().isApprox(Eigen::Array3d(-0.1, 0, -0.2)));
  REQUIRE(bbox.getMax().isApprox(Eigen::Array3d(1.3, 1.4, 1.2)));
}

TEST_CASE("Distance to points", "[bbox, geometry]") {
  Ramuh::BoundingBox3 bbox(Eigen::Array3d(0, 0, 0),
                           Eigen::Array3d(1.2, 1.4, 1));

  REQUIRE(bbox.computeDistanceToPoint(Eigen::Array3d(-0.1, 0.5, 0.5)) ==
          Approx(0.1));
  REQUIRE(bbox.computeDistanceToPoint(Eigen::Array3d(-0.1, -0.1, 0.5)) ==
          Approx(std::sqrt(0.02)));
  REQUIRE(bbox.computeDistanceToPoint(Eigen::Array3d(-0.1, -0.1, -0.1)) ==
          Approx(std::sqrt(0.03)));

  REQUIRE(bbox.computeDistanceToPoint(Eigen::Array3d(1.3, 0.5, 0.5)) ==
          Approx(0.1));
  REQUIRE(bbox.computeDistanceToPoint(Eigen::Array3d(1.3, 1.5, 0.5)) ==
          Approx(std::sqrt(0.02)));
  REQUIRE(bbox.computeDistanceToPoint(Eigen::Array3d(1.3, 1.5, 1.1)) ==
          Approx(std::sqrt(0.03)));

  REQUIRE(bbox.computeDistanceToPoint(Eigen::Array3d(1.2, 0.7, 0.5)) ==
          Approx(0));
}

TEST_CASE("Merging bounding boxes 3D", "[bbox, geometry]") {
  Ramuh::BoundingBox3 bbox(0, 1);

  SECTION("Merge with point") {
    bbox.mergeBoxes(Eigen::Array3d(0.1, 0.5, 0.8));
    REQUIRE(bbox.getMin().isApprox(Eigen::Array3d(0, 0, 0)));
    REQUIRE(bbox.getMax().isApprox(Eigen::Array3d(1, 1, 1)));

    bbox.mergeBoxes(Eigen::Array3d(-0.1, 0.5, -0.25));
    REQUIRE(bbox.getMin().isApprox(Eigen::Array3d(-0.1, 0, -0.25)));
    REQUIRE(bbox.getMax().isApprox(Eigen::Array3d(1, 1, 1)));

    bbox.mergeBoxes(Eigen::Array3d(1.4, 0.5, 0.5));
    REQUIRE(bbox.getMin().isApprox(Eigen::Array3d(-0.1, 0, -0.25)));
    REQUIRE(bbox.getMax().isApprox(Eigen::Array3d(1.4, 1, 1)));
  }

  bbox = Ramuh::BoundingBox3(0, 1);
  SECTION("Merge with box") {
    bbox.mergeBoxes(Ramuh::BoundingBox3(Eigen::Array3d(0.4, 0.5, 0.12),
                                        Eigen::Array3d(1.2, 0.7, 0.25)));
    REQUIRE(bbox.getMin().isApprox(Eigen::Array3d(0, 0, 0)));
    REQUIRE(bbox.getMax().isApprox(Eigen::Array3d(1.2, 1, 1)));

    bbox.mergeBoxes(Ramuh::BoundingBox3(Eigen::Array3d(-0.4, -0.5, 0.12),
                                        Eigen::Array3d(0, 0.3, 0.25)));
    REQUIRE(bbox.getMin().isApprox(Eigen::Array3d(-0.4, -0.5, 0)));
    REQUIRE(bbox.getMax().isApprox(Eigen::Array3d(1.2, 1, 1)));
  }
}