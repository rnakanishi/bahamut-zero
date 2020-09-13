#include <glad/glad.h>

#include <iostream>
#include <shaders/shader.hpp>

namespace Bismarck {
Shader::Shader() {
  _vertexShader = glCreateShader(GL_VERTEX_SHADER);
  _fragShader = glCreateShader(GL_FRAGMENT_SHADER);
  _shaderProgram = glCreateProgram();
}

Shader::~Shader() {
  glDeleteProgram(_shaderProgram);
}

unsigned int Shader::getShaderProgram() {
  return _shaderProgram;
}

void Shader::checkShaderErrors(unsigned int shaderId) {
  int success;
  char infoLog[512];
  glGetShaderiv(shaderId, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(shaderId, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }
}

void Shader::checkProgramErrors(unsigned int programId) {
  int success;
  char infoLog[512];
  glGetProgramiv(programId, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(programId, 512, NULL, infoLog);
    std::cerr << "ERROR::SHADER::PROGRAM::COMPILATION_FAILED\n"
              << infoLog << std::endl;
  }
}

void Shader::compileShader() {
  _vertexSource =
      "#version 330 core\n"
      "layout (location = 0) in vec3 aPos;\n"
      "void main()\n"
      "{\n"
      "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
      "}\0";

  _fragSource =
      "#version 330 core\n"
      "out vec4 FragColor;\n"
      "void main()\n"
      "{\n"
      "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
      "}\n\0";

  glShaderSource(_vertexShader, 1, &_vertexSource, NULL);
  glCompileShader(_vertexShader);
  checkShaderErrors(_vertexShader);

  glShaderSource(_fragShader, 1, &_fragSource, NULL);
  glCompileShader(_fragShader);
  checkShaderErrors(_fragShader);

  glAttachShader(_shaderProgram, _vertexShader);
  glAttachShader(_shaderProgram, _fragShader);
  glLinkProgram(_shaderProgram);
  checkProgramErrors(_shaderProgram);

  glUseProgram(_shaderProgram);

  glDeleteShader(_vertexShader);
  glDeleteShader(_fragShader);
}
}  // namespace Bismarck
