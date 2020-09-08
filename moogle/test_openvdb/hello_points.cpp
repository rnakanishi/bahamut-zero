#include <iostream>
#include <vector>

#include <openvdb/openvdb.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointCount.h>

int main(int argc, char const *argv[]) {
  openvdb::initialize();

  // Creating vector with points
  std::vector<openvdb::Vec3R> positions;
  positions.push_back(openvdb::Vec3R(0, 1, 0));
  positions.push_back(openvdb::Vec3R(1.5, 2.5, 1));
  positions.push_back(openvdb::Vec3R(-1, 6, -2));
  positions.push_back(openvdb::Vec3R(1.1, 1.25, 0.06));

  openvdb::points::PointAttributeVector<openvdb::Vec3R> positionsWrapper(
      positions);

  int pointPerVoxel = 8;
  float voxelSize =
      openvdb::points::computeVoxelSize(positionsWrapper, pointPerVoxel);

  std::cout << "Voxel size: " << voxelSize << std::endl;

  openvdb::math::Transform::Ptr transform =
      openvdb::math::Transform::createLinearTransform(voxelSize);
  openvdb::points::PointDataGrid::Ptr grid =
      openvdb::points::createPointDataGrid<openvdb::points::NullCodec,
                                           openvdb::points::PointDataGrid>(
          positions, *transform);
  grid->setName("Points");
  openvdb::io::File("mypoints.vdb").write({grid});

  return 0;
}
