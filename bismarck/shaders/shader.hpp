#ifndef __BISMARCK_SHADER_HPP__
#define __BISMARCK_SHADER_HPP__

#include <string>

namespace Bismarck {
class Shader {
 public:
  Shader();
  ~Shader();

  virtual void compileShader();

  virtual unsigned int getShaderProgram();

  void checkShaderErrors(unsigned int shaderId);

  void checkProgramErrors(unsigned int programId);

  /**
   * @brief This method calls the shader program to use itself. If the shader is
   * not compiled, it tries to compile and then bind it to be used.
   *
   */
  void render();

 protected:
  std::string _shaderName;
  bool _isCompiled;

  unsigned int _vertexShader;
  unsigned int _fragShader;
  unsigned int _shaderProgram;
};
}  // namespace Bismarck

#endif