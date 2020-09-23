#ifndef __GARUDA_OBJ_LOADER_HPP__
#define __GARUDA_OBJ_LOADER_HPP__

#include <garuda_utils/file_loader.hpp>
#include <renderable/triangle_mesh.hpp>

namespace Garuda {
class ObjLoader : public FileLoader {
 public:
  void loadFile(std::string filename, TriangleMesh &object) override;
};
}  // namespace Garuda

#endif