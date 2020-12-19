#ifndef __GARUDA_TEXTURE_OBJECT_HPP__
#define __GARUDA_TEXTURE_OBJECT_HPP__

#include <renderable/render_object.hpp>
#include <string>

namespace Garuda {
class TextureObject {
 public:
  TextureObject();

  void setFilename(std::string& path);

  std::string& getFilename();

  // TODO: change to shaded pointer
  void setImage(unsigned char** image, int width, int height, int channels);

  int getWidth();

  int getHeight();

  unsigned char* getImageData();

 protected:
  std::string _texPath;
  unsigned char* _image;
  int _width, _height, _channels;
  bool _isSet;
};

}  // namespace Garuda
#endif