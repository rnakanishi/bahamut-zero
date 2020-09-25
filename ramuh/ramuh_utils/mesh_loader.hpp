#ifndef __RAMUH_MESH_LOADER_HPP__
#define __RAMUH_MESH_LOADER_HPP__

#include <geometry/face_vertex_mesh.hpp>
#include <string>

namespace Ramuh {
class MeshLoader {
 public:
  MeshLoader();

  static void loadObj(const std::string& filename, FaceVertexMesh& mesh);
};
}  // namespace Ramuh

#endif
