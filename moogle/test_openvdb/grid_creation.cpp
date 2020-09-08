#include <iostream>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <string>

using namespace openvdb;

int main(int argc, char const *argv[]) {
  openvdb::initialize();
  std::string filename = "results/mygrids.vdb";

  // Creating a grid and writing to a file
  FloatGrid::Ptr grid =
      tools::createLevelSetSphere<FloatGrid>(50.0, Vec3f(1.5, 2, 3), 0.5, 4.0);
  grid->insertMeta("radius", FloatMetadata(50.0));
  grid->setName("LevelSetSphere");
  io::File(filename).write({grid});

  // Reading the file and modifying its content
  io::File file(filename);
  file.open();

  GridBase::Ptr baseGrid;
  for (io::File::NameIterator nameIter = file.beginName();
       nameIter != file.endName(); ++nameIter) {
    if (nameIter.gridName() == "LevelSetSphere") {
      baseGrid = file.readGrid(nameIter.gridName());
    } else {
      std::cout << "Skipping grid " << nameIter.gridName() << std::endl;
    }
  }

  return 0;
}
