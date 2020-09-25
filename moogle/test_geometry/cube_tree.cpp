#define CATCH_CONFIG_CONSOLE_WIDTH 300
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <exceptions/exception.hpp>
#include <geometry/cube_tree.hpp>
#include <geometry/face_vertex_mesh.hpp>
#include <ramuh_utils/mesh_loader.hpp>
#include <string>

// TEST_CASE("Mesh loader", "[mesh, triangles, geometry]") {
//   SECTION("Cube object") {
//     Ramuh::FaceVertexMesh mesh;
//     Ramuh::MeshLoader loader;

//     std::string meshPath =
//         "/home/rnakanishi/git/bahamut-zero/kingMoogle/meshes/cube.obj";
//     loader.loadObj(meshPath, mesh);
//     Ramuh::CubeTree tree(mesh);
//     REQUIRE(tree.countLeavesWithVertices() == 8);
//   }
//   SECTION("Triangle object") {
//     Ramuh::FaceVertexMesh mesh;
//     Ramuh::MeshLoader loader;

//     std::string meshPath =
//         "/home/rnakanishi/git/bahamut-zero/kingMoogle/meshes/triangle.obj";
//     loader.loadObj(meshPath, mesh);
//     Ramuh::CubeTree tree(mesh);
//     REQUIRE(tree.countLeavesWithVertices() == 3);
//   }
// }

TEST_CASE("Node level retrieval", "[cubeTree, geometry]") {
  Ramuh::CubeTree tree;

  REQUIRE(tree.getNodeLevel(2) == 1);
  REQUIRE(tree.getNodeLevel(8) == 1);
  REQUIRE(tree.getNodeLevel(9) == 2);
  REQUIRE(tree.getNodeLevel(25) == 2);
  REQUIRE(tree.getNodeLevel(72) == 2);
  REQUIRE(tree.getNodeLevel(73) == 3);
  REQUIRE(tree.getNodeLevel(584) == 3);
  REQUIRE(tree.getNodeLevel(585) == 4);
  REQUIRE(tree.getNodeLevel(791) == 4);
  REQUIRE(tree.getNodeLevel(4680) == 4);
  REQUIRE(tree.getNodeLevel(4681) == 5);
  REQUIRE(tree.getNodeLevel(12755) == 5);
  REQUIRE(tree.getNodeLevel(37448) == 5);
}

TEST_CASE("Closest point to triangle", "[mesh, triangles, geometry]") {
  Ramuh::FaceVertexMesh mesh;
  Ramuh::MeshLoader loader;

  std::string meshPath =
      "/home/rnakanishi/git/bahamut-zero/kingMoogle/meshes/triangle.obj";

  loader.loadObj(meshPath, mesh);

  Ramuh::CubeTree tree(mesh);

  REQUIRE(tree.findClosestPoint(Eigen::Array3f(0, 3.4, -1))
              .isApprox(Eigen::Array3f(0, 1, 0)));
  REQUIRE(tree.findClosestPoint(Eigen::Array3f(-2, -1.7, 0))
              .isApprox(Eigen::Array3f(-1, -1, 0)));
  REQUIRE(tree.findClosestPoint(Eigen::Array3f(4, -6.1, 2))
              .isApprox(Eigen::Array3f(1, -1, 0)));
  REQUIRE(tree.findClosestPoint(Eigen::Array3f(0.3, -0.3, 0))
              .isApprox(Eigen::Array3f(0.3, -0.3, 0)));
  REQUIRE(tree.findClosestPoint(Eigen::Array3f(-1, 0.3, 0.4))
              .isApprox(Eigen::Array3f(-0.48, 0.04, 0)));
  REQUIRE(tree.findClosestPoint(Eigen::Array3f(1, -0.2, -0.5))
              .isApprox(Eigen::Array3f(0.68, -0.36, 0)));
}

TEST_CASE("Closest point to cube", "[mesh, triangles, geometry]") {
  Ramuh::FaceVertexMesh mesh;
  Ramuh::MeshLoader loader;

  std::string meshPath =
      "/home/rnakanishi/git/bahamut-zero/kingMoogle/meshes/cube.obj";

  loader.loadObj(meshPath, mesh);

  Ramuh::CubeTree tree(mesh);

  REQUIRE(tree.findClosestPoint(Eigen::Array3f(0.25, 3.4, 0.7))
              .isApprox(Eigen::Array3f(0.25, 1.0, 0.7)));
  REQUIRE(tree.findClosestPoint(Eigen::Array3f(2.25, 0.4, 0.7))
              .isApprox(Eigen::Array3f(1.0, 0.4, 0.7)));
  // Inside
  REQUIRE(tree.findClosestPoint(Eigen::Array3f(0.25, 0.8, 0.7))
              .isApprox(Eigen::Array3f(0.25, 1.0, 0.7)));

  REQUIRE(tree.findClosestPoint(Eigen::Array3f(1.25, 3.4, 0.7))
              .isApprox(Eigen::Array3f(1.0, 1.0, 0.7)));
  REQUIRE(tree.findClosestPoint(Eigen::Array3f(1.25, 0.7, -2.4))
              .isApprox(Eigen::Array3f(1.0, 0.7, -1)));

  REQUIRE(tree.findClosestPoint(Eigen::Array3f(-5.25, -3.4, 2.1))
              .isApprox(Eigen::Array3f(-1, -1, 1)));
}