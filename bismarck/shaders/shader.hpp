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

 protected:
  unsigned int _vertexShader;
  unsigned int _fragShader;
  unsigned int _shaderProgram;

  char* _vertexSource;
  char* _fragSource;
};
}  // namespace Bismarck

#endif