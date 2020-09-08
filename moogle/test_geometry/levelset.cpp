#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <geometry/levelset.hpp>
#include <utils/exception.hpp>

TEST_CASE("Levelset surface location", "[levelset]") {
  Palkia::Levelset2 levelset;
  levelset.initializeLevelset(Palkia::Levelset2::Shape::CIRCLE);
  auto h = levelset.getSpacing();

  SECTION("Horizontal surface") {
    for (size_t cellId = 0; cellId < levelset.getCellCount(); cellId++) {
      auto cellPosition = levelset.getCellPosition(cellId);

      try {
        auto surfacePosition = levelset.findSurfacePosition(
            cellId, Palkia::Levelset2::Direction::HORIZONTAL);

        double x = surfacePosition[0];
        double y = surfacePosition[1];
        CHECK(x * x + y * y - 0.25 == Approx(0.).margin(h[0] * h[0]));
      } catch (Arceus::ArceusException exception) {
      }
    }
  }

  SECTION("Vertical surface") {
    for (size_t cellId = 0; cellId < levelset.getCellCount(); cellId++) {
      auto cellPosition = levelset.getCellPosition(cellId);

      try {
        auto surfacePosition = levelset.findSurfacePosition(
            cellId, Palkia::Levelset2::Direction::VERTICAL);
        double x = surfacePosition[0];
        double y = surfacePosition[1];
        CHECK(x * x + y * y - 0.25 == Approx(0.).margin(h[1] * h[1]));
      } catch (Arceus::ArceusException exception) {
      }
    }
  }
}
