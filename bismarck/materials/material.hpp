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
  virtual void render();

  /**
   * @brief Create a small interface component to help the user set material
   * values
   *
   */
  virtual void showUI();

 protected:
  Shader _shader;
};

}  // namespace Bismarck

#endif