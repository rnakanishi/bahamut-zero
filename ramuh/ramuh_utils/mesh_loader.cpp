#include <tiny_obj_loader.h>
#include <iostream>
#include <ramuh_utils/mesh_loader.hpp>

namespace Ramuh {
MeshLoader::MeshLoader() {}

void MeshLoader::loadObj(const std::string& filename, FaceVertexMesh& mesh) {
  std::string warning, error;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  tinyobj::attrib_t attributes;

  std::string baseDir(filename);
  int lastSlash = baseDir.find_last_of("/\\");

  bool ret = tinyobj::LoadObj(
      &attributes, &shapes, &materials, &warning, &error, filename.c_str(),
      baseDir.substr(0, lastSlash).c_str(), true, false);
  bool hasNormal = (!attributes.normals.empty()) ? true : false;

  // Load coordinates to MeshObject
  for (int i = 0; i < attributes.vertices.size() / 3; i++) {
    mesh.addVertex(Eigen::Array3f(attributes.vertices[3 * i + 0],
                                  attributes.vertices[3 * i + 1],
                                  attributes.vertices[3 * i + 2]));
    if (hasNormal) {
      mesh.addNormal(Eigen::Vector3f(attributes.normals[3 * i + 0],
                                     attributes.normals[3 * i + 1],
                                     attributes.normals[3 * i + 2]));
    }
  }

  for (auto shape : shapes)
    for (int i = 0; i < shapes[0].mesh.indices.size() / 3; i++) {
      mesh.addFace(Eigen::Array3i(shape.mesh.indices[3 * i + 0].vertex_index,
                                  shape.mesh.indices[3 * i + 1].vertex_index,
                                  shape.mesh.indices[3 * i + 2].vertex_index));
    }

  //   for (auto vertex : attributes.vertices) {
  // mesh.addVertex(Eigen::Array3f(vertex[0], vertex[1], vertex[2]));
  //   }
}
}  // namespace Ramuh
