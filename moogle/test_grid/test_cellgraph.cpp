#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <grid/cell_graph.hpp>

TEST_CASE("Cellgraph instancing", "[cellgraph]") {
  Ramuh::CellGraph2 cellgraph;

  auto newNodes = cellgraph.refineNode(1);
  SECTION("First refinement") {
    REQUIRE(newNodes.size() == 4);
    REQUIRE(newNodes[0] == 2);
    REQUIRE(newNodes[1] == 3);
    REQUIRE(newNodes[2] == 4);
    REQUIRE(newNodes[3] == 5);
  }

  newNodes = cellgraph.refineNode(2);
  SECTION("Second refinement") {
    REQUIRE(newNodes.size() == 4);
    REQUIRE(newNodes[0] == 1);
    REQUIRE(newNodes[1] == 6);
    REQUIRE(newNodes[2] == 7);
    REQUIRE(newNodes[3] == 8);
  }

  newNodes = cellgraph.refineNode(3);
  SECTION("Second refinement 2") {
    REQUIRE(newNodes.size() == 4);
    REQUIRE(newNodes[0] == 2);
    REQUIRE(newNodes[1] == 9);
    REQUIRE(newNodes[2] == 10);
    REQUIRE(newNodes[3] == 11);
  }
}

TEST_CASE("Neighbor checking", "[cellgraph]") {
  Ramuh::CellGraph2 cellgraph;

  auto newNodes = cellgraph.refineNode(1);
  SECTION("First refinement") {
    std::vector<size_t> neighbors;
    neighbors = cellgraph.findNodeNeighbors(2);
    REQUIRE(neighbors.size() == 2);
    for (auto id : neighbors) {
      REQUIRE((id == 3 || id == 4));
    }

    neighbors = cellgraph.findNodeNeighbors(3);
    REQUIRE(neighbors.size() == 2);
    for (auto id : neighbors) {
      REQUIRE((id == 2 || id == 5));
    }

    neighbors = cellgraph.findNodeNeighbors(4);
    REQUIRE(neighbors.size() == 2);
    for (auto id : neighbors) {
      REQUIRE((id == 2 || id == 5));
    }

    neighbors = cellgraph.findNodeNeighbors(5);
    REQUIRE(neighbors.size() == 2);
    for (auto id : neighbors) {
      REQUIRE((id == 3 || id == 4));
    }
  }

  newNodes = cellgraph.refineNode(2);
  SECTION("Second refinement") {
    std::vector<size_t> neighbors;
    neighbors = cellgraph.findNodeNeighbors(8);
    REQUIRE(neighbors.size() == 4);
    for (auto id : neighbors) {
      REQUIRE((id == 3 || id == 4 || id == 6 || id == 7));
    }

    neighbors = cellgraph.findNodeNeighbors(3);
    REQUIRE(neighbors.size() == 3);
    for (auto id : neighbors) {
      REQUIRE((id == 6 || id == 8 || id == 5));
    }

    neighbors = cellgraph.findNodeNeighbors(4);
    REQUIRE(neighbors.size() == 3);
    for (auto id : neighbors) {
      REQUIRE((id == 7 || id == 8 || id == 5));
    }
  }

  newNodes = cellgraph.refineNode(3);
  SECTION("Second refinement 2") {
    std::vector<size_t> neighbors;
    neighbors = cellgraph.findNodeNeighbors(10);
    REQUIRE(neighbors.size() == 4);
    for (auto id : neighbors) {
      REQUIRE((id == 2 || id == 5 || id == 8 || id == 11));
    }

    neighbors = cellgraph.findNodeNeighbors(6);
    REQUIRE(neighbors.size() == 3);
    for (auto id : neighbors) {
      REQUIRE((id == 2 || id == 1 || id == 8));
    }

    neighbors = cellgraph.findNodeNeighbors(11);
    REQUIRE(neighbors.size() == 3);
    for (auto id : neighbors) {
      REQUIRE((id == 9 || id == 5 || id == 10));
    }

    neighbors = cellgraph.findNodeNeighbors(5);
    REQUIRE(neighbors.size() == 3);
    for (auto id : neighbors) {
      REQUIRE((id == 10 || id == 11 || id == 4));
    }
  }
}
