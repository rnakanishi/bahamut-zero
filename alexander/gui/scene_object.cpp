#include "scene_object.hpp"

namespace Alexander {
SceneObject::SceneObject() {}

std::vector<Eigen::Array3f> SceneObject::loadObjectVertices() {
  std::vector<Eigen::Array3f> object;

  object.emplace_back(Eigen::Array3f(0.5f, 0.5f, 0.0f));
  object.emplace_back(Eigen::Array3f(0.5f, -0.5f, 0.0f));
  object.emplace_back(Eigen::Array3f(-0.5f, -0.5f, 0.0f));
  object.emplace_back(Eigen::Array3f(-0.5f, 0.5f, 0.0f));

  return object;
}

std::vector<Eigen::Array3i> SceneObject::loadObjectFaces() {
  std::vector<Eigen::Array3i> faces;
  faces.emplace_back(0, 1, 2);
  faces.emplace_back(0, 2, 3);
  return faces;
}

}  // namespace Alexander
