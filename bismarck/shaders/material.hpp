#ifndef __BISMARCK_MATERIAL_HPP__
#define __BISMARCK_MATERIAL_HPP__

#include <shaders/shader.hpp>

namespace Bismarck {

class Material {
 public:
  Material();

  // TODO: create UI element for imgui

  void compileShader();

  /**
   * @brief This method calls the shader program to use itself. If the shader is
   * not compiled, it tries to compile and then bind it to be used.
   *
   */
  void render();

 private:
  Shader _shader;
};

}  // namespace Bismarck

#endif