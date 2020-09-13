#ifndef __GARUDA_RENDER_OBJECT_HPP__
#define __GARUDA_RENDER_OBJECT_HPP__

#include <shaders/shader.hpp>

namespace Garuda {

class RenderObject {
 public:
  RenderObject();

  ~RenderObject();

  /**
   * @brief
   *
   */
  virtual void render() = 0;

  virtual void sendDataToBuffer() = 0;

  virtual void bindShader(Bismarck::Shader shader);

 protected:
  unsigned int _vbo, _ebo, _vao;
  Bismarck::Shader _shader;
};

}  // namespace Garuda

#endif