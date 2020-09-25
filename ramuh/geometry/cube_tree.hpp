#ifndef __RAMUH_CUBE_TREE_HPP__
#define __RAMUH_CUBE_TREE_HPP__

#include <geometry/bounding_box.hpp>
#include <geometry/face_vertex_mesh.hpp>
#include <limits>
#include <map>
#include <memory>

namespace Ramuh {

/**
 * @brief This class take as input a Mesh object and computes its bounding
 * sphere to build the tree. The tree nodes contains the mesh polygons inside
 * them and is used to auxiliate queries to find its closest points
 *
 */
class CubeTree {
 public:
  CubeTree();
  CubeTree(const Ramuh::FaceVertexMesh& mesh);

  std::vector<int> getNodeChildren(int nodeId);

  Eigen::Array3f findClosestPoint(
      Eigen::Array3f point,
      float maxDistance = std::numeric_limits<float>::max());

  /**
   * @brief For all leaves in the tree, check if they contain a vertex and
   * increase the counter if so.
   *
   * @return int total leaves with vertices
   */
  int countLeavesWithVertices();

 protected:
  int _treeDepth;
  std::shared_ptr<FaceVertexMesh> _mesh;

  std::map<int, std::set<int>> _verticesInNode;
  std::vector<Ramuh::BoundingBox3> _boxes;
  std::vector<bool> _hasVertex;
};
}  // namespace Ramuh

#endif