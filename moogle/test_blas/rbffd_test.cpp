#define CATCH_CONFIG_MAIN
#include <Eigen/Sparse>
#include <blas/rbf_finite_difference.hpp>
#include <catch2/catch.hpp>
#include <cmath>
#include <functional>
#include <geometry/levelset.hpp>
#include <iostream>
#include <random>
#include <utils/exception.hpp>

TEST_CASE("Laplacian weights RBF-FD", "[rbf, finite_difference]") {
  Giratina::DifferentialRBF2 rbf;

  Eigen::Array2d target;
  std::vector<Eigen::Array2d> points;
  std::vector<double> values;

  std::function<std::vector<double>(std::vector<Eigen::Array2d>)> function =
      [](std::vector<Eigen::Array2d> points) {
        std::vector<double> values;
        for (auto &&point : points) {
          values.emplace_back(point.x() + point.y());
        }
        return values;
      };

  SECTION("Standard cross distribution") {
    // Add the points
    target = Eigen::Array2d(0, 0);
    points.emplace_back(0.0, 0.0);
    points.emplace_back(0.5, 0.0);
    points.emplace_back(-0.5, 0.0);
    points.emplace_back(0.0, 0.5);
    points.emplace_back(0.0, -0.5);

    rbf.addSamplePoint(points, function(points));
    std::vector<double> weights;
    try {
      weights = rbf.computeLaplacianWeightsAt(target);
    } catch (Arceus::ArceusException exception) {
      std::cerr << exception.what() << '\n';
    }

    REQUIRE_FALSE(weights[0] == 0);
    REQUIRE(weights.size() == 5);
    REQUIRE(weights[1] == Approx(weights[2]));
    REQUIRE(weights[1] == Approx(weights[3]));
    REQUIRE(weights[1] == Approx(weights[4]));
    REQUIRE(-weights[0] ==
            Approx(weights[1] + weights[2] + weights[3] + weights[4]));
  }

  // clear all previous data for the next test
  rbf.clearSamples();
  points.clear();
  values.clear();

  SECTION("Standard cross distribution target off origin") {
    // Add the points
    target = Eigen::Array2d(0, 0);
    points.emplace_back(3.0, 7.0);
    points.emplace_back(3.5, 7.0);
    points.emplace_back(2.5, 7.0);
    points.emplace_back(3.0, 7.5);
    points.emplace_back(3.0, 6.5);

    rbf.addSamplePoint(points, function(points));
    std::vector<double> weights;
    try {
      weights = rbf.computeLaplacianWeightsAt(target);
    } catch (Arceus::ArceusException exception) {
      std::cerr << exception.what() << '\n';
    }

    REQUIRE_FALSE(weights[0] == 0);
    REQUIRE(weights.size() == 5);
    REQUIRE(weights[1] == Approx(weights[2]));
    REQUIRE(weights[1] == Approx(weights[3]));
    REQUIRE(weights[1] == Approx(weights[4]));
    REQUIRE(-weights[0] ==
            Approx(weights[1] + weights[2] + weights[3] + weights[4]));
  }

  // clear all previous data for the next test
  rbf.clearSamples();
  points.clear();
  values.clear();

  SECTION("Circular equally spaced shape sample points") {
    int nPoints = 12;
    double degree = 2 * M_PI / nPoints;
    target = Eigen::Array2d(0, 0);
    points.emplace_back(0.0, 0.0);
    for (size_t i = 0; i < nPoints; i++) {
      points.emplace_back(std::cos(i * degree), std::sin(i * degree));
    }

    rbf.addSamplePoint(points, function(points));
    std::vector<double> weights;
    try {
      weights = rbf.computeLaplacianWeightsAt(target);
    } catch (Arceus::ArceusException exception) {
      std::cerr << exception.what() << '\n';
      exit(exception.getErrorNumber());
    }

    REQUIRE(weights.size() == points.size());
    double weightSum = weights[1];
    for (size_t i = 2; i < points.size(); i++) {
      REQUIRE(weights[1] == Approx(weights[i]));
      weightSum += weights[i];
    }

    REQUIRE_FALSE(weights[0] == 0);
    REQUIRE(-weights[0] == Approx(weightSum));
  }
}

// ################################################################
// ================================================================
// ----------------------------------------------------------------
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
TEST_CASE("Laplacian RBFFD interpolated value", "[rbf, finite_difference]") {
  Giratina::DifferentialRBF2 rbf;
  Eigen::Array2d target;
  std::vector<Eigen::Array2d> points;
  std::vector<double> values;
  std::vector<double> weights;

  std::function<double(Eigen::Array2d)> myfunc = [](Eigen::Array2d point) {
    double x = point.x();
    double y = point.y();

    return std::exp(x) * std::cos(y * y);
  };

  std::function<std::vector<double>(std::vector<Eigen::Array2d>)>
      functionVector = [myfunc](std::vector<Eigen::Array2d> points) {
        std::vector<double> values;
        for (auto &&point : points) {
          values.emplace_back(myfunc(point));
        }
        return values;
      };

  SECTION("Standard cross distribution") {
    // Add the points
    target = Eigen::Array2d(0, 0);
    points.emplace_back(0.0, 0.0);
    points.emplace_back(0.125, 0.0);
    points.emplace_back(-0.125, 0.0);
    points.emplace_back(0.0, 0.125);
    points.emplace_back(0.0, -0.125);
    values = functionVector(points);

    rbf.addSamplePoint(points, functionVector(points));
    try {
      weights = rbf.computeLaplacianWeightsAt(target);
    } catch (Arceus::ArceusException exception) {
      std::cerr << exception.what() << '\n';
    }

    double targetValue = myfunc(target);
    double numerical = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
      numerical += weights[i] * values[i];
    }

    REQUIRE(numerical == Approx(targetValue).epsilon(0.125 * 0.125));
  }

  SECTION("Circular equally spaced shape sample points") {
    rbf.clearSamples();
    points.clear();
    values.clear();
    weights.clear();

    int nPoints = 12;
    double degree = 2 * M_PI / nPoints;
    double h = 0.125;
    target = Eigen::Array2d(0, 0);
    points.emplace_back(0.0, 0.0);
    for (size_t i = 0; i < nPoints; i++) {
      points.emplace_back(std::cos(i * degree) * h, std::sin(i * degree) * h);
    }
    values = functionVector(points);

    rbf.addSamplePoint(points, values);
    try {
      weights = rbf.computeLaplacianWeightsAt(target);
    } catch (Arceus::ArceusException exception) {
      std::cerr << exception.what() << '\n';
      exit(exception.getErrorNumber());
    }

    double targetValue = myfunc(target);
    double numerical = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
      numerical += weights[i] * values[i];
    }
    REQUIRE(numerical == Approx(targetValue).epsilon(h * h));
  }

  SECTION("Random scattered points (4)") {
    rbf.clearSamples();
    points.clear();
    values.clear();
    weights.clear();
    std::srand(404);
    int nPoints = 4;
    double h = 0.125;
    points.emplace_back(0., 0.);
    for (size_t i = 0; i < nPoints; i++) {
      double x = std::rand() % 1000000;
      double y = std::rand() % 1000000;
      x = x * 2 * h / 1000000 - h;
      y = y * 2 * h / 1000000 - h;
      points.emplace_back(x, y);
    }
    values = functionVector(points);

    rbf.addSamplePoint(points, values);
    try {
      weights = rbf.computeLaplacianWeightsAt(target);
    } catch (Arceus::ArceusException exception) {
      std::cerr << exception.what() << '\n';
      exit(exception.getErrorNumber());
    }
    double targetValue = myfunc(target);
    double numerical = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
      numerical += weights[i] * values[i];
    }
    REQUIRE(numerical == Approx(targetValue).epsilon(2 * h * h));
  }

  SECTION("Random scattered points (25)") {
    rbf.clearSamples();
    points.clear();
    values.clear();
    weights.clear();
    std::srand(404);
    int nPoints = 25;
    double h = 0.125;
    points.emplace_back(0., 0.);
    for (size_t i = 0; i < nPoints; i++) {
      double x = std::rand() % 1000000;
      double y = std::rand() % 1000000;
      x = x * 2 * h / 1000000 - h;
      y = y * 2 * h / 1000000 - h;
      points.emplace_back(x, y);
    }
    values = functionVector(points);

    rbf.addSamplePoint(points, values);
    try {
      weights = rbf.computeLaplacianWeightsAt(target);
    } catch (Arceus::ArceusException exception) {
      std::cerr << exception.what() << '\n';
      exit(exception.getErrorNumber());
    }
    double targetValue = myfunc(target);
    double numerical = 0.0;
    for (size_t i = 0; i < points.size(); i++) {
      numerical += weights[i] * values[i];
    }
    REQUIRE(numerical == Approx(targetValue).epsilon(h * h));
  }
}

// ################################################################
// ================================================================
// ----------------------------------------------------------------
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
