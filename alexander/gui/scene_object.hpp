#ifndef __ALEXANDER_SCENE_OBJECT_HPP__
#define __ALEXANDER_SCENE_OBJECT_HPP__

#include <Eigen/Dense>

namespace Alexander {
class SceneObject {
 public:
  SceneObject();

  std::vector<Eigen::ArrayXf> loadObjectVertices();

  std::vector<Eigen::ArrayXi> loadObjectFaces();

 private:
};
}  // namespace Alexander

#endif