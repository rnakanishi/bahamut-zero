#ifndef __GARUDA_RENDER_OBJECT_HPP__
#define __GARUDA_RENDER_OBJECT_HPP__

#include <materials/material.hpp>
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

 protected:
  unsigned int _vbo, _ebo, _vao;
  Bismarck::Material _material;
};

}  // namespace Garuda

#endif