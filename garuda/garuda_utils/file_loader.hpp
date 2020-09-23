#ifndef __GARUDA_FILE_LOADER_HPP__
#define __GARUDA_FILE_LOADER_HPP__

#include <renderable/triangle_mesh.hpp>
#include <string>

namespace Garuda {

class FileLoader {
 public:
  virtual void loadFile(std::string filename, TriangleMesh&) = 0;
};
}  // namespace Garuda
#endif
