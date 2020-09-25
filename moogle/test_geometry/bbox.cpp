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

TEST_CASE("Triangle intersection", "[bbox, triangles, geometry]") {
  Ramuh::BoundingBox3 bbox(0, 1);
  std::vector<Eigen::Array3d> triangle;

  {
    triangle.emplace_back(0.6, 0.5, -0.6);
    triangle.emplace_back(0.6, 0.5, 0.4);
    triangle.emplace_back(1.6, 0.5, 0.4);
    REQUIRE(bbox.doesIntersectWithTriangle(triangle) == true);
    triangle.clear();
  }
  {
    triangle.emplace_back(-0.65, 0.6, 0.5);
    triangle.emplace_back(-0.65, 0.6, 1.5);
    triangle.emplace_back(0.35, 0.6, 1.5);
    REQUIRE(bbox.doesIntersectWithTriangle(triangle) == false);
    triangle.clear();
  }
  {
    triangle.emplace_back(-0.5, 0.828606, 0.116978);
    triangle.emplace_back(-0.5, 1.47139, 0.883022);
    triangle.emplace_back(0.5, 1.47139, 0.883022);
    REQUIRE(bbox.doesIntersectWithTriangle(triangle) == false);
    triangle.clear();
  }
  {
    triangle.emplace_back(0.399608, 0.48483, 1.32035);
    triangle.emplace_back(1.27808, 0.184452, 0.948779);
    triangle.emplace_back(0.800392, -0.38483, 0.279649);
    REQUIRE(bbox.doesIntersectWithTriangle(triangle) == true);
    triangle.clear();
  }
  {
    triangle.emplace_back(0.0159708, 0.527874, 0.724267);
    triangle.emplace_back(0.723652, 0.362888, 0.0372683);
    triangle.emplace_back(0.723652, 0.362888, 0.0372683);
    REQUIRE(bbox.doesIntersectWithTriangle(triangle) == true);
    triangle.clear();
  }
  {
    triangle.emplace_back(-0.439505, 1.52507, 2.67478);
    triangle.emplace_back(2.23974, 0.914007, 0.0850533);
    triangle.emplace_back(-0.147922, -1.1578, -1.89628);
    REQUIRE(bbox.doesIntersectWithTriangle(triangle) == true);
    triangle.clear();
  }
  {
    triangle.emplace_back(-2.76616, 0.108621, 5.07756);
    triangle.emplace_back(2.75186, -1.14989, -0.256099);
    triangle.emplace_back(0.754494, 2.58028, -3.20266);
    REQUIRE(bbox.doesIntersectWithTriangle(triangle) == true);
    triangle.clear();
  }
  {
    Ramuh::BoundingBox3 bbox(1.5, 2.5);
    triangle.emplace_back(1.5 + -2.76616, 1.5 + 0.108621, 1.5 + 5.07756);
    triangle.emplace_back(1.5 + 2.75186, 1.5 + -1.14989, 1.5 + -0.256099);
    triangle.emplace_back(1.5 + 0.754494, 1.5 + 2.58028, 1.5 + -3.20266);
    REQUIRE(bbox.doesIntersectWithTriangle(triangle) == true);
    triangle.clear();
  }
}