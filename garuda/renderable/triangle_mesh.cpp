#include <glad/glad.h>
#include <iostream>
#include <renderable/triangle_mesh.hpp>
#include <shaders/shader.hpp>

namespace Garuda {

TriangleMesh::TriangleMesh() : TriangleMesh(std::vector<Eigen::Array3f>()) {}

TriangleMesh::TriangleMesh(std::vector<Eigen::Array3f> positions)
    : RenderObject() {
  vertexVectorLabels["positions"] = 0;
  vertexVectorPropeties.emplace_back(std::vector<float>());
  vectorPropertyDimension.emplace_back(3);
  for (auto&& data : positions) {
    vertexVectorPropeties[0].insert(vertexVectorPropeties[0].end(), data.data(),
                                    data.data() + data.size());
  }

  _modelMatrix.setIdentity();
  _normalMatrix.setIdentity();
}

TriangleMesh::~TriangleMesh() {}

void TriangleMesh::render() {
  _material.render();

  glBindVertexArray(_vao);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
  glBindVertexArray(0);
}

void TriangleMesh::assignMaterial(Bismarck::Material newMaterial) {
  _material = newMaterial;
  _material.compileShader();
}

Garuda::TextureObject& TriangleMesh::getTexture() {
  return _texture;
}

}  // namespace Garuda
