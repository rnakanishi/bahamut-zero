#ifndef __GARUDA_TEXTURE_OBJECT_HPP__
#define __GARUDA_TEXTURE_OBJECT_HPP__

#include <renderable/render_object.hpp>
#include <string>

namespace Garuda {
class TextureObject : public RenderObject {
 public:
  TextureObject();

  void setFilename(std::string filename);

 protected:
  std::string _filename;
  bool _isSet;
};
}  // namespace Garuda

#endif