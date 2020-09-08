#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <iostream>
#include <openvdb/openvdb.h>
#include <openvdb/tools/ChangeBackground.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <string>

using namespace openvdb;

TEST_CASE("OpenVDB IO", "[openvdb]") {
  openvdb::initialize();
  std::string filename = "results/mygrids.vdb";

  SECTION("Writing grid") {
    // Creating a grid and writing to a file
    FloatGrid::Ptr grid = tools::createLevelSetSphere<FloatGrid>(
        50.0, Vec3f(1.5, 2, 3), 0.5, 4.0);
    grid->insertMeta("radius", FloatMetadata(50.0));
    grid->setName("LevelSetSphere");
    io::File(filename).write({grid});
  }

  GridBase::Ptr baseGrid;
  SECTION("Reading grid file") {
    // Reading the file and modifying its content
    io::File file(filename);
    file.open();

    // Iterate through all grids in the file and take the one with the desired
    // name
    for (io::File::NameIterator nameIter = file.beginName();
         nameIter != file.endName(); ++nameIter) {
      if (nameIter.gridName() == "LevelSetSphere") {
        baseGrid = file.readGrid(nameIter.gridName());
      } else {
        std::cout << "Skipping grid " << nameIter.gridName() << std::endl;
      }
    }
    file.close();

    // Change grid values and background
    FloatGrid::Ptr grid = gridPtrCast<FloatGrid>(baseGrid);
    const float outside = grid->background();
    const float width = 2.0 * outside;
    std::cout << "Outside value " << outside << std::endl;

    // Values inside get the distance to the surface
    for (FloatGrid::ValueOnIter iter = grid->beginValueOn(); iter; ++iter) {
      float dist = iter.getValue();
      iter.setValue((outside - dist) / width);
    }

    // Values outside are set to 1
    for (FloatGrid::ValueOffIter iter = grid->beginValueOff(); iter; ++iter) {
      if (iter.getValue() < 0.0) {
        iter.setValue(1.0);
        iter.setValueOff();
      }
    }
    tools::changeBackground(grid->tree(), 0.5);
    // io::File(filename).write({grid});
  }
}

TEST_CASE("OpenVDB Stream", "[openvdb]") { openvdb::initialize(); }