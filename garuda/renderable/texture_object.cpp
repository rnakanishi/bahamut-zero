#include <materials/texture_material.hpp>
#include <renderable/texture_object.hpp>

namespace Garuda {

TextureObject::TextureObject() {
  // _material = Bismarck::TextureMaterial();
  _isSet = false;
}

void TextureObject::setFilename(std::string filename) {
  _filename = filename;
  _isSet = true;
}

}  // namespace Garuda
