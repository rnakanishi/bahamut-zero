#ifndef __BISMARCK_TEXTURE_HPP__
#define __BISMARCK_TEXTURE_HPP__

#include <string>

namespace Bismarck {
class Texture {
 public:
  Texture();

  void setTextureFilename(std::string& path);

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
};
}  // namespace Bismarck

#endif