#include "scene_object.hpp"

namespace Alexander {
SceneObject::SceneObject() {}

std::vector<Eigen::ArrayXf> SceneObject::loadObjectVertices() {
  std::vector<Eigen::ArrayXf> object;

  object.emplace_back(Eigen::Array3f(0.5f, 0.5f, 0.0f));
  object.emplace_back(Eigen::Array3f(0.5f, -0.5f, 0.0f));
  object.emplace_back(Eigen::Array3f(-0.5f, -0.5f, 0.0f));
  object.emplace_back(Eigen::Array3f(-0.5f, 0.5f, 0.0f));

  return object;
}

std::vector<Eigen::ArrayXi> SceneObject::loadObjectFaces() {
  std::vector<Eigen::ArrayXi> faces;
  faces.emplace_back(Eigen::Array3i(0, 1, 2));
  faces.emplace_back(Eigen::Array3i(0, 2, 3));
  return faces;
}

}  // namespace Alexander
