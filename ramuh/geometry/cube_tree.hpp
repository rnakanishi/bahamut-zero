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
 * box to build the tree. The tree nodes contains the mesh polygons inside
 * them and is used to auxiliate queries to find its closest points
 *
 * For every child of a given cell, checks if a surface face crosses the cell,
 * adding it to an internal vector that manages the cells.
 *
 */
class CubeTree {
 public:
  CubeTree();
  CubeTree(const Ramuh::FaceVertexMesh& mesh);

  /**
   * @brief Given a node ID, retrieves all the vertices that are inside that
   * node.
   *
   * @param nodeId
   * @return std::vector<int>
   */
  std::vector<int> getNodeChildren(int nodeId) const;

  /**
   * @brief Get the Node Level by recursively multiplying by 8 and accumulating
   * the sum
   *
   * @param nodeId
   * @return int node level
   */
  int getNodeLevel(int nodeId) const;

  /**
   * @brief Given a point query, finds the cloasest point that relies on the
   * surface.
   *
   * This method builds a priority queue to check for distances. This is done
   * because not always the closest cell to the point is the solution, as
   * triangles may fully cross a cell, or the query point may be very close to
   * cell face and the neighboring cell contains a tangent face that is closest
   * to query.
   *
   * @param point
   * @param maxDistance
   * @return Eigen::Array3f
   */
  Eigen::Array3f findClosestPoint(
      Eigen::Array3f point,
      float maxDistance = std::numeric_limits<float>::max()) const;

  /**
   * @brief For all leaves in the tree, check if they contain a vertex and
   * increase the counter if so.
   *
   * @return int total leaves with vertices
   */
  int countLeavesWithVertices() const;

 protected:
  int _treeDepth;
  std::shared_ptr<FaceVertexMesh> _mesh;

  std::map<int, std::set<int>> _verticesInNode, _facesInNode;
  std::vector<Ramuh::BoundingBox3> _boxes;
  std::vector<bool> _hasFace;
};
}  // namespace Ramuh

#endif