#define CATCH_CONFIG_CONSOLE_WIDTH 300
#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <exceptions/exception.hpp>
#include <geometry/face_vertex_mesh.hpp>
#include <ramuh_utils/mesh_loader.hpp>
#include <string>

TEST_CASE("Cube mesh loader", "[mesh, triangles, geometry]") {
  Ramuh::FaceVertexMesh mesh;
  Ramuh::MeshLoader loader;

  std::string meshPath =
      "/home/rnakanishi/git/bahamut-zero/kingMoogle/meshes/cube.obj";

  loader.loadObj(meshPath, mesh);

  REQUIRE(mesh.getVerticesCount() == 8);
  REQUIRE(mesh.getFacesCount() == 12);
}

TEST_CASE("Distance to triangle", "[mesh, triangles, geometry]") {
  Ramuh::FaceVertexMesh mesh;
  Ramuh::MeshLoader loader;

  std::string meshPath =
      "/home/rnakanishi/git/bahamut-zero/kingMoogle/meshes/cube.obj";

  loader.loadObj(meshPath, mesh);

  REQUIRE(mesh.computeDistanceToFace(0, Eigen::Array3f(-1, 1.5, -1)) ==
          Approx(0.5));
  REQUIRE(mesh.computeDistanceToFace(0, Eigen::Array3f(1.5, 0.5, 0.5)) ==
          Approx(0.5));
}

TEST_CASE("Closest point", "[mesh, triangles, geometry]") {
  Ramuh::FaceVertexMesh mesh;
  Ramuh::MeshLoader loader;

  std::string meshPath =
      "/home/rnakanishi/git/bahamut-zero/kingMoogle/meshes/triangle.obj";

  loader.loadObj(meshPath, mesh);

  REQUIRE(mesh.findClosestPointToFace(0, Eigen::Array3f(0, 3.4, -1))
              .isApprox(Eigen::Array3f(0, 1, 0)));
  REQUIRE(mesh.findClosestPointToFace(0, Eigen::Array3f(-2, -1.7, 0))
              .isApprox(Eigen::Array3f(-1, -1, 0)));
  REQUIRE(mesh.findClosestPointToFace(0, Eigen::Array3f(4, -6.1, 2))
              .isApprox(Eigen::Array3f(1, -1, 0)));
  REQUIRE(mesh.findClosestPointToFace(0, Eigen::Array3f(0.3, -0.3, 0))
              .isApprox(Eigen::Array3f(0.3, -0.3, 0)));
  REQUIRE(mesh.findClosestPointToFace(0, Eigen::Array3f(-1, 0.3, 0.4))
              .isApprox(Eigen::Array3f(-0.48, 0.04, 0)));
  REQUIRE(mesh.findClosestPointToFace(0, Eigen::Array3f(1, -0.2, -0.5))
              .isApprox(Eigen::Array3f(0.68, -0.36, 0)));
}