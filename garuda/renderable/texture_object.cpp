#include <materials/texture_material.hpp>
#include <renderable/texture_object.hpp>

namespace Garuda {

TextureObject::TextureObject() {
  // _material = Bismarck::TextureMaterial();
  _isSet = false;
}

void TextureObject::setFilename(std::string& filename) {
  _texPath = filename;
  _isSet = true;
}

int TextureObject::getWidth() {
  return _width;
}
int TextureObject::getHeight() {
  return _height;
}
unsigned char* TextureObject::getImageData() {
  return _image;
}

void TextureObject::setImage(unsigned char** image,
                             int width,
                             int height,
                             int channels) {
  _image = *image;
  _width = width;
  _height = height;
  _channels = channels;
}

std::string& TextureObject::getFilename() {
  return _texPath;
}

}  // namespace Garuda
