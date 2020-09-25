#ifndef __RAMUH_FACE_VERTEX_MESH_HPP__
#define __RAMUH_FACE_VERTEX_MESH_HPP__

#include <Eigen/Dense>
#include <geometry/bounding_box.hpp>
#include <set>

namespace Ramuh {
class FaceVertexMesh {
 public:
  FaceVertexMesh();

  /**
   * @brief Given a point in the space, add this point as a mesh vertex. This
   * method also allows a vector of points as parameter. The returning value
   * corresponds to the id of the new inserted vertex, or a vector of ids if
   * multiple points are given.
   *
   * @param point vertex to be added
   * @return int corresponding vertex id
   */
  int addVertex(Eigen::Array3f point);
  std::vector<int> addVertices(std::vector<Eigen::Array3f> points);

  /**
   * @brief Receives an array of integers corresponding to the face vertices
   * ids. The structure use those ids to build a face and returns its new id.
   * The method can also receive a vector of arrays, corresponding to multiple
   * faces.
   *
   * @param faceVertices vertices ids that build the face
   * @return int new face ID
   */
  int addFace(Eigen::ArrayXi faceVertices);
  std::vector<int> addFaces(std::vector<Eigen::ArrayXi> facesVertices);

  /**
   * @brief Receives a normal vector in space and add it to the structure
   *
   * @param normal
   * @return int
   */
  int addNormal(Eigen::Vector3f normal);

  /**
   * @brief Get the total of vertices in the structure
   *
   * @return int total vertices counts
   */
  int getVerticesCount();

  /**
   * @brief Get the total count of normals in the structure. This value may
   * differ from vertices count, as normals can be reused by vertices
   *
   * @return int
   */
  int getNormalsCount();

  /**
   * @brief Get the total face count in the structure
   *
   * @return int
   */
  int getFacesCount();

  /**
   * @brief Given a vertex Id, retrieve all the faces incident to that vertex
   *
   * @param vertexId
   * @return std::vector<int>
   */
  std::vector<int> getVertexFaces(int vertexId);

  /**
   * @brief Given a face id, retrieve all the vertices that form this face
   *
   * @param faceId
   * @return std::vector<int>
   */
  std::vector<int> getFaceVertices(int faceId);

  /**
   * @brief Get the Vertex Coordinate from the vertexId
   *
   * @param vertexId
   * @return Eigen::Array3f
   */
  Eigen::Array3f getVertexCoordinate(int vertexId);

  /**
   * @brief return the bounding box of the mesh
   *
   * @return BoundingBox3
   */
  const BoundingBox3& computeBoundingBox();

  /**
   * @brief Get the Vertices Position vector. This is a read only, so the user
   * can make parallel access to the vector as well.
   *
   * @return const std::vector<Eigen::Array3f>&
   */
  const std::vector<Eigen::Array3f>& getVerticesPositionVector();

  bool doesFaceContainPoint(int faceId, Eigen::Array3f point);

  /**
   * @brief Given a point query, compute the distance of the point to the face
   * with faceId. This method does not take into consideration the direction of
   * the face normal
   *
   * @param faceId
   * @param point
   * @return float
   */
  float computeDistanceToFace(int faceId, Eigen::Array3f point);

  Eigen::Array3f findClosestPointToFace(int faceId, Eigen::Array3f point);

 protected:
  std::vector<Eigen::Array3f> _verticesPosition;
  std::vector<Eigen::Vector3f> _normals;
  std::vector<int> _verticesNormals;
  std::vector<int> _faceNormals;
  std::vector<std::set<int>> _verticesFaces;
  std::vector<std::set<int>> _faceVertices;

  BoundingBox3 _bbox;
};
}  // namespace Ramuh

#endif